package main

import (
	"container/heap"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"math/rand"
	"net/http"
	"os"
	"strconv"
	"sync"
	"time"
)

// キャンパス中心から切り出す半径（メートル）※ここを変えると範囲を調整できます
const campusRadiusMeters = 350.0

type LayerRule struct {
	Name     string
	Drivable bool
	SpeedKmh float64
}

var layerRules = map[string]LayerRule{
	"#38bdf8": {Name: "一般車道", Drivable: true, SpeedKmh: 35.0},
	"#f97316": {Name: "幹線道路", Drivable: true, SpeedKmh: 45.0},
	"#a855f7": {Name: "特設バイパス", Drivable: true, SpeedKmh: 40.0},
	"#eab308": {Name: "徐行ルート", Drivable: true, SpeedKmh: 20.0},
	"#22c55e": {Name: "歩行者専用(車不可)", Drivable: false, SpeedKmh: 0.0},
}

type TwinNode struct {
	ID       string  `json:"id"`
	Lat      float64 `json:"lat"`
	Lon      float64 `json:"lon"`
	Color    string  `json:"color"`
	IsCustom bool    `json:"isCustom"`
}

type TwinEdge struct {
	ID       int         `json:"id"`
	U        string      `json:"u"`
	V        string      `json:"v"`
	Length   float64     `json:"length"`
	Color    string      `json:"color"`
	Name     string      `json:"name"`
	Blocked  bool        `json:"blocked"`
	Oneway   bool        `json:"oneway"`
	IsCustom bool        `json:"isCustom"`
	Geometry [][]float64 `json:"geometry,omitempty"` // 正確な道路形状 [lat, lon] の配列
}

type TwinVehicle struct {
	ID             int      `json:"id"`
	Route          []string `json:"-"`
	SegIndex       int      `json:"-"`
	ProgressMeters float64  `json:"-"`
	Lat            float64  `json:"lat"`
	Lon            float64  `json:"lon"`
	SpeedKmh       float64  `json:"speedKmh"`
	StatusColor    string   `json:"statusColor"`
}

type TwinWorldState struct {
	Center       map[string]float64   `json:"center"`
	IsRunning    bool                 `json:"isRunning"`
	TargetVehCnt int                  `json:"targetVehCnt"`
	MapRevision  int                  `json:"mapRevision"`
	Nodes        map[string]*TwinNode `json:"nodes"`
	Edges        []*TwinEdge          `json:"edges"`
	Vehicles     []*TwinVehicle       `json:"vehicles"`
}

type TwinCommand struct {
	Action    string   `json:"action"`
	Delta     int      `json:"delta"`
	SubAction string   `json:"subAction"`
	NodeIDs   []string `json:"nodeIds"`
	EdgeIDs   []int    `json:"edgeIds"`
}

type TwinEngine struct {
	mu           sync.RWMutex
	centerLat    float64
	centerLon    float64
	nodes        map[string]*TwinNode
	edges        []*TwinEdge
	vehicles     []*TwinVehicle
	isRunning    bool
	targetVehCnt int
	mapRevision  int
	nextVehID    int
}

var twinEngine *TwinEngine

func haversineMeters(lat1, lon1, lat2, lon2 float64) float64 {
	const R = 6371000.0
	dLat := (lat2 - lat1) * math.Pi / 180.0
	dLon := (lon2 - lon1) * math.Pi / 180.0
	a := math.Sin(dLat/2)*math.Sin(dLat/2) +
		math.Cos(lat1*math.Pi/180.0)*math.Cos(lat2*math.Pi/180.0)*math.Sin(dLon/2)*math.Sin(dLon/2)
	return 2 * R * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
}

func isNodeDrivable(n *TwinNode) bool {
	if n == nil {
		return false
	}
	rule, ok := layerRules[n.Color]
	if !ok {
		return true
	}
	return rule.Drivable
}

func isEdgeDrivable(e *TwinEdge, nodes map[string]*TwinNode) bool {
	if e == nil || e.Blocked {
		return false
	}
	rule, ok := layerRules[e.Color]
	if ok && !rule.Drivable {
		return false
	}
	return isNodeDrivable(nodes[e.U]) && isNodeDrivable(nodes[e.V])
}

func getEdgeSpeedMs(e *TwinEdge) float64 {
	if e == nil {
		return 35.0 / 3.6
	}
	rule, ok := layerRules[e.Color]
	if !ok || rule.SpeedKmh <= 0 {
		return 35.0 / 3.6
	}
	return rule.SpeedKmh / 3.6
}

// --- A* 経路探索 ---
type astarItem struct {
	nodeID string
	fScore float64
	index  int
}

type astarPQ []*astarItem

func (pq astarPQ) Len() int           { return len(pq) }
func (pq astarPQ) Less(i, j int) bool { return pq[i].fScore < pq[j].fScore }
func (pq astarPQ) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].index = i
	pq[j].index = j
}
func (pq *astarPQ) Push(x any) {
	item := x.(*astarItem)
	item.index = len(*pq)
	*pq = append(*pq, item)
}
func (pq *astarPQ) Pop() any {
	old := *pq
	n := len(old)
	item := old[n-1]
	old[n-1] = nil
	*pq = old[:n-1]
	return item
}

type adjLink struct {
	to   string
	cost float64
}

func (eng *TwinEngine) buildAdjacency() map[string][]adjLink {
	adj := make(map[string][]adjLink, len(eng.nodes))
	for _, e := range eng.edges {
		if !isEdgeDrivable(e, eng.nodes) {
			continue
		}
		speedMs := getEdgeSpeedMs(e)
		cost := e.Length / speedMs
		adj[e.U] = append(adj[e.U], adjLink{to: e.V, cost: cost})
		if !e.Oneway {
			adj[e.V] = append(adj[e.V], adjLink{to: e.U, cost: cost})
		}
	}
	return adj
}

func (eng *TwinEngine) findPathAStar(startID, goalID string, adj map[string][]adjLink) []string {
	startNode := eng.nodes[startID]
	goalNode := eng.nodes[goalID]
	if startNode == nil || goalNode == nil || startID == goalID {
		return nil
	}

	const maxSpeedMs = 45.0 / 3.6
	pq := &astarPQ{}
	heap.Init(pq)
	heap.Push(pq, &astarItem{
		nodeID: startID,
		fScore: haversineMeters(startNode.Lat, startNode.Lon, goalNode.Lat, goalNode.Lon) / maxSpeedMs,
	})

	cameFrom := make(map[string]string)
	gScore := map[string]float64{startID: 0}
	closed := make(map[string]bool)

	for pq.Len() > 0 {
		curr := heap.Pop(pq).(*astarItem).nodeID
		if curr == goalID {
			path := []string{goalID}
			for curr != startID {
				curr = cameFrom[curr]
				path = append(path, curr)
			}
			for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
				path[i], path[j] = path[j], path[i]
			}
			return path
		}
		if closed[curr] {
			continue
		}
		closed[curr] = true

		for _, link := range adj[curr] {
			if closed[link.to] {
				continue
			}
			tentativeG := gScore[curr] + link.cost
			if prevG, exists := gScore[link.to]; !exists || tentativeG < prevG {
				cameFrom[link.to] = curr
				gScore[link.to] = tentativeG
				toNode := eng.nodes[link.to]
				h := haversineMeters(toNode.Lat, toNode.Lon, goalNode.Lat, goalNode.Lon) / maxSpeedMs
				heap.Push(pq, &astarItem{nodeID: link.to, fScore: tentativeG + h})
			}
		}
	}
	return nil
}

func (eng *TwinEngine) drivableNodeIDs() []string {
	ids := make([]string, 0, len(eng.nodes))
	for id, n := range eng.nodes {
		if isNodeDrivable(n) {
			ids = append(ids, id)
		}
	}
	return ids
}

func (eng *TwinEngine) createRouteFrom(startID string, ids []string, adj map[string][]adjLink) []string {
	if len(ids) < 2 {
		return nil
	}
	for attempt := 0; attempt < 25; attempt++ {
		endID := ids[rand.Intn(len(ids))]
		if endID == startID {
			continue
		}
		path := eng.findPathAStar(startID, endID, adj)
		if len(path) >= 2 {
			return path
		}
	}
	return nil
}

func (eng *TwinEngine) syncVehicleCountLocked() {
	ids := eng.drivableNodeIDs()
	if len(ids) < 2 {
		return
	}
	adj := eng.buildAdjacency()

	for len(eng.vehicles) < eng.targetVehCnt {
		startID := ids[rand.Intn(len(ids))]
		path := eng.createRouteFrom(startID, ids, adj)
		if len(path) < 2 {
			break
		}
		n0 := eng.nodes[path[0]]
		eng.nextVehID++
		eng.vehicles = append(eng.vehicles, &TwinVehicle{
			ID:          eng.nextVehID,
			Route:       path,
			SegIndex:    0,
			Lat:         n0.Lat,
			Lon:         n0.Lon,
			SpeedKmh:    28.0 + rand.Float64()*8.0,
			StatusColor: "#22c55e",
		})
	}
	if len(eng.vehicles) > eng.targetVehCnt {
		eng.vehicles = eng.vehicles[:eng.targetVehCnt]
	}
}

func (eng *TwinEngine) rerouteAllLocked() {
	ids := eng.drivableNodeIDs()
	if len(ids) < 2 {
		return
	}
	adj := eng.buildAdjacency()
	for _, v := range eng.vehicles {
		var startID string
		if v.SegIndex < len(v.Route) && isNodeDrivable(eng.nodes[v.Route[v.SegIndex]]) {
			startID = v.Route[v.SegIndex]
		} else {
			startID = ids[rand.Intn(len(ids))]
		}
		path := eng.createRouteFrom(startID, ids, adj)
		if len(path) >= 2 {
			v.Route = path
			v.SegIndex = 0
			v.ProgressMeters = 0
		}
	}
}

func (eng *TwinEngine) findEdgeBetween(u, v string) *TwinEdge {
	for _, e := range eng.edges {
		if (e.U == u && e.V == v) || (!e.Oneway && e.U == v && e.V == u) {
			return e
		}
	}
	return nil
}

// ポリライン形状に沿って進捗率(0.0~1.0)に対応する正確な緯度経度を計算
func interpolateAlongEdge(e *TwinEdge, uNode, vNode *TwinNode, ratio float64) (float64, float64) {
	if ratio <= 0 {
		return uNode.Lat, uNode.Lon
	}
	if ratio >= 1 {
		return vNode.Lat, vNode.Lon
	}
	if e == nil || len(e.Geometry) < 2 {
		return uNode.Lat + (vNode.Lat-uNode.Lat)*ratio, uNode.Lon + (vNode.Lon-uNode.Lon)*ratio
	}

	pts := e.Geometry
	// 逆向き走行の場合は配列を反転して計算
	if e.U != uNode.ID {
		rev := make([][]float64, len(pts))
		for i := range pts {
			rev[i] = pts[len(pts)-1-i]
		}
		pts = rev
	}

	segLens := make([]float64, len(pts)-1)
	var total float64
	for i := 0; i < len(pts)-1; i++ {
		d := haversineMeters(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
		segLens[i] = d
		total += d
	}
	if total <= 0 {
		return uNode.Lat + (vNode.Lat-uNode.Lat)*ratio, uNode.Lon + (vNode.Lon-uNode.Lon)*ratio
	}

	targetDist := total * ratio
	var acc float64
	for i := 0; i < len(segLens); i++ {
		if acc+segLens[i] >= targetDist || i == len(segLens)-1 {
			localRatio := 0.0
			if segLens[i] > 0 {
				localRatio = (targetDist - acc) / segLens[i]
			}
			lat := pts[i][0] + (pts[i+1][0]-pts[i][0])*localRatio
			lon := pts[i][1] + (pts[i+1][1]-pts[i][1])*localRatio
			return lat, lon
		}
		acc += segLens[i]
	}
	return vNode.Lat, vNode.Lon
}

// 20Hz IDM物理演算ステップ
func (eng *TwinEngine) stepSimulation(dt float64) {
	eng.mu.Lock()
	defer eng.mu.Unlock()

	if !eng.isRunning || len(eng.vehicles) == 0 {
		return
	}

	var adjCache map[string][]adjLink
	var idsCache []string

	for i, v := range eng.vehicles {
		if v.SegIndex >= len(v.Route)-1 {
			continue
		}
		uID := v.Route[v.SegIndex]
		vID := v.Route[v.SegIndex+1]
		n1 := eng.nodes[uID]
		n2 := eng.nodes[vID]
		if n1 == nil || n2 == nil {
			continue
		}

		curEdge := eng.findEdgeBetween(uID, vID)
		desiredMs := getEdgeSpeedMs(curEdge)
		segDist := math.Max(haversineMeters(n1.Lat, n1.Lon, n2.Lat, n2.Lon), 5.0)
		if curEdge != nil && curEdge.Length > 0 {
			segDist = curEdge.Length
		}
		speedMs := v.SpeedKmh / 3.6

		minGap := 999.0
		leadSpeedMs := desiredMs
		for j, other := range eng.vehicles {
			if i == j || other.SegIndex >= len(other.Route)-1 {
				continue
			}
			if other.Route[other.SegIndex] == uID && other.Route[other.SegIndex+1] == vID {
				gap := other.ProgressMeters - v.ProgressMeters
				if gap > 0 && gap < minGap {
					minGap = gap
					leadSpeedMs = other.SpeedKmh / 3.6
				}
			}
		}

		const s0 = 5.0
		const T = 1.4
		const aMax = 1.8
		const bComf = 2.5
		deltaV := speedMs - leadSpeedMs
		sStar := s0 + math.Max(0, speedMs*T+(speedMs*deltaV)/(2*math.Sqrt(aMax*bComf)))
		interaction := 0.0
		if minGap < 100.0 {
			ratio := sStar / math.Max(minGap, 1.5)
			interaction = ratio * ratio
		}
		acc := aMax * (1.0 - math.Pow(speedMs/desiredMs, 4) - interaction)
		speedMs = math.Max(0.5, math.Min(desiredMs, speedMs+acc*dt))
		v.SpeedKmh = speedMs * 3.6
		v.ProgressMeters += speedMs * dt

		if v.ProgressMeters >= segDist {
			v.ProgressMeters -= segDist
			v.SegIndex++
			if v.SegIndex >= len(v.Route)-1 {
				if adjCache == nil {
					adjCache = eng.buildAdjacency()
					idsCache = eng.drivableNodeIDs()
				}
				startID := idsCache[rand.Intn(len(idsCache))]
				path := eng.createRouteFrom(startID, idsCache, adjCache)
				if len(path) >= 2 {
					v.Route = path
					v.SegIndex = 0
					v.ProgressMeters = 0
				}
			}
		}

		if v.SegIndex < len(v.Route)-1 {
			curU := eng.nodes[v.Route[v.SegIndex]]
			curV := eng.nodes[v.Route[v.SegIndex+1]]
			if curU != nil && curV != nil {
				activeEdge := eng.findEdgeBetween(curU.ID, curV.ID)
				curLen := math.Max(haversineMeters(curU.Lat, curU.Lon, curV.Lat, curV.Lon), 5.0)
				if activeEdge != nil && activeEdge.Length > 0 {
					curLen = activeEdge.Length
				}
				ratio := math.Min(1.0, v.ProgressMeters/curLen)
				v.Lat, v.Lon = interpolateAlongEdge(activeEdge, curU, curV, ratio)
			}
		}

		if v.SpeedKmh < 15.0 {
			v.StatusColor = "#ef4444"
		} else if v.SpeedKmh < 26.0 {
			v.StatusColor = "#f59e0b"
		} else {
			v.StatusColor = "#22c55e"
		}
	}
}

func parseGeometry(m map[string]any) [][]float64 {
	rawGeo := pickAny(m, "geometry", "coords", "coordinates", "path")
	arr, ok := rawGeo.([]any)
	if !ok || len(arr) < 2 {
		return nil
	}
	pts := make([][]float64, 0, len(arr))
	for _, item := range arr {
		switch p := item.(type) {
		case []any:
			if len(p) >= 2 {
				a, ok1 := toFloat(p[0])
				b, ok2 := toFloat(p[1])
				if ok1 && ok2 {
					// GeoJSON形式 [lon, lat] の場合は [lat, lon] に正規化
					if a > 100 && b < 90 {
						pts = append(pts, []float64{b, a})
					} else {
						pts = append(pts, []float64{a, b})
					}
				}
			}
		case map[string]any:
			lat, ok1 := toFloat(pickAny(p, "lat", "y"))
			lon, ok2 := toFloat(pickAny(p, "lon", "x", "lng"))
			if ok1 && ok2 {
				pts = append(pts, []float64{lat, lon})
			}
		}
	}
	if len(pts) < 2 {
		return nil
	}
	return pts
}

func loadGraphFromFile() (*TwinEngine, error) {
	paths := []string{
		"../colab-engine/campus_graph.json",
		"colab-engine/campus_graph.json",
		"campus_graph.json",
	}
	var raw []byte
	var err error
	for _, p := range paths {
		raw, err = os.ReadFile(p)
		if err == nil {
			log.Printf("[TwinEngine] 地図データ読み込み成功: %s", p)
			break
		}
	}
	if err != nil {
		return nil, err
	}

	var parsed map[string]any
	if err := json.Unmarshal(raw, &parsed); err != nil {
		return nil, err
	}

	eng := &TwinEngine{
		centerLat:    35.687,
		centerLon:    139.327,
		nodes:        make(map[string]*TwinNode),
		edges:        make([]*TwinEdge, 0),
		vehicles:     make([]*TwinVehicle, 0),
		isRunning:    true,
		targetVehCnt: 45,
		mapRevision:  1,
	}

	if meta, ok := parsed["metadata"].(map[string]any); ok {
		if c, ok := meta["center"].(map[string]any); ok {
			if lat, ok := toFloat(c["lat"]); ok {
				eng.centerLat = lat
			}
			if lon, ok := toFloat(c["lon"]); ok {
				eng.centerLon = lon
			}
		}
	}

	// 1. 中心から campusRadiusMeters 以内のノードだけを抽出して範囲を狭める
	if rawNodes, ok := parsed["nodes"].([]any); ok {
		for _, item := range rawNodes {
			m, ok := item.(map[string]any)
			if !ok {
				continue
			}
			id := fmt.Sprintf("%v", pickAny(m, "id", "ID", "node_id"))
			lat, okLat := toFloat(pickAny(m, "lat", "y", "Lat"))
			lon, okLon := toFloat(pickAny(m, "lon", "x", "Lon", "lng"))
			if id == "" || !okLat || !okLon {
				continue
			}
			if haversineMeters(lat, lon, eng.centerLat, eng.centerLon) > campusRadiusMeters {
				continue
			}
			color := "#38bdf8"
			if c, ok := m["color"].(string); ok && c != "" {
				color = c
			}
			eng.nodes[id] = &TwinNode{ID: id, Lat: lat, Lon: lon, Color: color}
		}
	}

	// 2. 両端が抽出範囲内にあるエッジだけを追加（正確な形状Geometryも保持）
	connectedNodes := make(map[string]bool)
	if rawEdges, ok := parsed["edges"].([]any); ok {
		for idx, item := range rawEdges {
			m, ok := item.(map[string]any)
			if !ok {
				continue
			}
			u := fmt.Sprintf("%v", pickAny(m, "u", "source", "from"))
			v := fmt.Sprintf("%v", pickAny(m, "v", "target", "to"))
			n1, n2 := eng.nodes[u], eng.nodes[v]
			if n1 == nil || n2 == nil {
				continue
			}
			dist, okD := toFloat(pickAny(m, "length", "dist", "distance"))
			if !okD || dist <= 0 {
				dist = haversineMeters(n1.Lat, n1.Lon, n2.Lat, n2.Lon)
			}
			bc, _ := toFloat(pickAny(m, "betweenness", "centrality"))
			color := "#38bdf8"
			if c, ok := m["color"].(string); ok && c != "" {
				color = c
			} else if bc > 0.06 {
				color = "#f97316"
			} else if bc > 0.02 {
				color = "#eab308"
			}
			name := "キャンパス道路"
			if nm, ok := m["name"].(string); ok && nm != "" {
				name = nm
			}
			oneway, _ := m["oneway"].(bool)
			geo := parseGeometry(m)

			eng.edges = append(eng.edges, &TwinEdge{
				ID:       idx,
				U:        u,
				V:        v,
				Length:   math.Max(dist, 5.0),
				Color:    color,
				Name:     name,
				Oneway:   oneway,
				Geometry: geo,
			})
			connectedNodes[u] = true
			connectedNodes[v] = true
		}
	}

	// 孤立ノード（道路に繋がっていない点）を掃除
	for id := range eng.nodes {
		if !connectedNodes[id] {
			delete(eng.nodes, id)
		}
	}

	eng.syncVehicleCountLocked()
	return eng, nil
}

func pickAny(m map[string]any, keys ...string) any {
	for _, k := range keys {
		if val, exists := m[k]; exists && val != nil {
			return val
		}
	}
	return nil
}

func toFloat(v any) (float64, bool) {
	switch t := v.(type) {
	case float64:
		return t, !math.IsNaN(t)
	case int:
		return float64(t), true
	case string:
		f, err := strconv.ParseFloat(t, 64)
		return f, err == nil
	default:
		return 0, false
	}
}

func initTwinEngine() {
	eng, err := loadGraphFromFile()
	if err != nil {
		log.Printf("[TwinEngine] 初期化エラー: %v", err)
		return
	}
	twinEngine = eng

	go func() {
		ticker := time.NewTicker(50 * time.Millisecond)
		defer ticker.Stop()
		for range ticker.C {
			twinEngine.stepSimulation(0.05)
		}
	}()

	http.HandleFunc("/api/twin/state", handleTwinState)
	http.HandleFunc("/api/twin/command", handleTwinCommand)
	log.Printf("[TwinEngine] キャンパス限定モード稼働開始 (Radius: %.0fm, Nodes: %d, Edges: %d, Vehicles: %d)",
		campusRadiusMeters, len(eng.nodes), len(eng.edges), len(eng.vehicles))
}

func handleTwinState(w http.ResponseWriter, r *http.Request) {
	if twinEngine == nil {
		http.Error(w, "Engine not initialized", http.StatusInternalServerError)
		return
	}
	twinEngine.mu.RLock()
	state := TwinWorldState{
		Center:       map[string]float64{"lat": twinEngine.centerLat, "lon": twinEngine.centerLon},
		IsRunning:    twinEngine.isRunning,
		TargetVehCnt: twinEngine.targetVehCnt,
		MapRevision:  twinEngine.mapRevision,
		Nodes:        twinEngine.nodes,
		Edges:        twinEngine.edges,
		Vehicles:     twinEngine.vehicles,
	}
	twinEngine.mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(state)
}

func handleTwinCommand(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost || twinEngine == nil {
		http.Error(w, "Invalid request", http.StatusBadRequest)
		return
	}
	var cmd TwinCommand
	if err := json.NewDecoder(r.Body).Decode(&cmd); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	twinEngine.mu.Lock()
	switch cmd.Action {
	case "set_vehicle_count":
		next := twinEngine.targetVehCnt + cmd.Delta
		if next < 5 {
			next = 5
		}
		if next > 200 {
			next = 200
		}
		twinEngine.targetVehCnt = next
		twinEngine.syncVehicleCountLocked()

	case "toggle_pause":
		twinEngine.isRunning = !twinEngine.isRunning

	case "area_action":
		edgeSet := make(map[int]bool)
		for _, id := range cmd.EdgeIDs {
			edgeSet[id] = true
		}
		nodeSet := make(map[string]bool)
		for _, id := range cmd.NodeIDs {
			nodeSet[id] = true
		}

		if cmd.SubAction == "block" || cmd.SubAction == "unblock" {
			shouldBlock := cmd.SubAction == "block"
			for _, e := range twinEngine.edges {
				if edgeSet[e.ID] {
					e.Blocked = shouldBlock
				}
			}
		} else if cmd.SubAction == "delete" {
			for id := range nodeSet {
				delete(twinEngine.nodes, id)
			}
			kept := make([]*TwinEdge, 0, len(twinEngine.edges))
			for _, e := range twinEngine.edges {
				if !edgeSet[e.ID] && !nodeSet[e.U] && !nodeSet[e.V] {
					kept = append(kept, e)
				}
			}
			twinEngine.edges = kept
		}
		twinEngine.mapRevision++
		twinEngine.rerouteAllLocked()

	case "reset_map":
		if fresh, err := loadGraphFromFile(); err == nil {
			twinEngine.nodes = fresh.nodes
			twinEngine.edges = fresh.edges
			twinEngine.vehicles = fresh.vehicles
			twinEngine.mapRevision++
		}
	}
	twinEngine.mu.Unlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{"status": "ok"})
}
