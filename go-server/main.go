package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math/rand"
	"net/http"
	"os"
	"runtime"
	"sort"
	"sync"
	"time"
)

type ParkingLot struct {
	ID       string `json:"id"`
	Name     string `json:"name"`
	Capacity int    `json:"capacity"`
	Occupied int    `json:"occupied"`
}

type Vehicle struct {
	ID          string `json:"id"`
	Plate       string `json:"plate"`
	IsApplied   bool   `json:"is_applied"`
	VehicleType string `json:"vehicle_type"`
	EntryGate   string `json:"entry_gate"`
}

type RankEntry struct {
	Name      string `json:"name"`
	Score     int    `json:"score"`
	Timestamp string `json:"timestamp"`
}

type ServerMetrics struct {
	Goroutines   int     `json:"goroutines"`
	MemoryAlloc  uint64  `json:"memory_kb"`
	TotalPackets int     `json:"total_packets"`
	LastGoTimeMs float64 `json:"last_go_ms"`
	LastPyTimeMs float64 `json:"last_py_ms"`
	LastAction   string  `json:"last_action"`
	EngineMode   string  `json:"engine_mode"`
}

type GameState struct {
	mu          sync.Mutex
	PlayerName  string                 `json:"player_name"`
	Score       int                    `json:"score"`
	Congestion  int                    `json:"congestion"`
	Parkings    map[string]*ParkingLot `json:"parkings"`
	CurrentCar  *Vehicle               `json:"current_car"`
	Leaderboard []RankEntry            `json:"leaderboard"`
	Metrics     ServerMetrics          `json:"metrics"`
	LastRoute   map[string]interface{} `json:"last_route"`
}

var (
	game = &GameState{
		PlayerName: "GUEST_STAFF",
		Score:      0,
		Congestion: 10,
		Parkings: map[string]*ParkingLot{
			"PARK_A": {ID: "PARK_A", Name: "A駐車場(申請者専用)", Capacity: 5, Occupied: 0},
			"PARK_B": {ID: "PARK_B", Name: "B駐車場(一般・搬入)", Capacity: 8, Occupied: 0},
		},
		Leaderboard: []RankEntry{
			{Name: "実行委員長", Score: 1200, Timestamp: "10:00:00"},
			{Name: "交通誘導班A", Score: 850, Timestamp: "10:15:30"},
		},
	}
	clients   = make(map[chan string]bool)
	clientsMu sync.Mutex
)

func getPythonEngineURL() string {
	if url := os.Getenv("PYTHON_ENGINE_URL"); url != "" {
		return url
	}
	return "http://127.0.0.1:8000"
}

func broadcastState() {
	var m runtime.MemStats
	runtime.ReadMemStats(&m)
	game.Metrics.Goroutines = runtime.NumGoroutine()
	game.Metrics.MemoryAlloc = m.Alloc / 1024

	data, _ := json.Marshal(game)
	msg := string(data)

	clientsMu.Lock()
	defer clientsMu.Unlock()
	for ch := range clients {
		select {
		case ch <- msg:
		default:
		}
	}
}

func generateVehicle() *Vehicle {
	plates := []string{"多摩 300 あ 12-34", "練馬 500 さ 56-78", "八王子 100 か 99-99", "品川 330 た 11-22"}
	gates := []string{"GATE_MAIN", "GATE_WEST"}
	types := []string{"CAR", "CAR", "TRUCK"}
	return &Vehicle{
		ID:          fmt.Sprintf("V-%d", rand.Intn(9000)+1000),
		Plate:       plates[rand.Intn(len(plates))],
		IsApplied:   rand.Intn(10) > 2,
		VehicleType: types[rand.Intn(len(types))],
		EntryGate:   gates[rand.Intn(len(gates))],
	}
}

// 約6秒ごとに自動出庫して空き枠を回復させるバックグラウンド処理
func startAutoExitLoop() {
	ticker := time.NewTicker(6 * time.Second)
	for range ticker.C {
		game.mu.Lock()
		changed := false
		for _, lot := range game.Parkings {
			if lot.Occupied > 0 {
				lot.Occupied--
				changed = true
			}
		}
		if changed {
			broadcastState()
		}
		game.mu.Unlock()
	}
}

func handleStream(w http.ResponseWriter, r *http.Request) {
	w.Header().Set("Content-Type", "text/event-stream")
	w.Header().Set("Cache-Control", "no-cache")
	w.Header().Set("Connection", "keep-alive")

	ch := make(chan string, 10)
	clientsMu.Lock()
	clients[ch] = true
	clientsMu.Unlock()

	defer func() {
		clientsMu.Lock()
		delete(clients, ch)
		clientsMu.Unlock()
	}()

	game.mu.Lock()
	initData, _ := json.Marshal(game)
	game.mu.Unlock()
	fmt.Fprintf(w, "data: %s\n\n", initData)
	w.(http.Flusher).Flush()

	for {
		select {
		case msg := <-ch:
			fmt.Fprintf(w, "data: %s\n\n", msg)
			w.(http.Flusher).Flush()
		case <-r.Context().Done():
			return
		}
	}
}

func handleState(w http.ResponseWriter, r *http.Request) {
	game.mu.Lock()
	defer game.mu.Unlock()
	if game.CurrentCar == nil {
		game.CurrentCar = generateVehicle()
	}
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(game)
}

func handleDispatch(w http.ResponseWriter, r *http.Request) {
	startGo := time.Now()

	var req struct {
		Action     string `json:"action"`
		PlayerName string `json:"player_name"`
	}
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}

	game.mu.Lock()
	if req.PlayerName != "" {
		game.PlayerName = req.PlayerName
	}
	if game.CurrentCar == nil {
		game.CurrentCar = generateVehicle()
	}
	game.Metrics.TotalPackets++

	car := game.CurrentCar
	msg := ""
	var routeData map[string]interface{}
	var pyTime float64

	if req.Action == "SUBMIT_RANK" {
		game.Leaderboard = append(game.Leaderboard, RankEntry{
			Name:      game.PlayerName,
			Score:     game.Score,
			Timestamp: time.Now().Format("15:04:05"),
		})
		sort.Slice(game.Leaderboard, func(i, j int) bool {
			return game.Leaderboard[i].Score > game.Leaderboard[j].Score
		})
		if len(game.Leaderboard) > 5 {
			game.Leaderboard = game.Leaderboard[:5]
		}
		msg = fmt.Sprintf("🏆 %s さんの記録 (%d pt) をランキングに登録しました！", game.PlayerName, game.Score)
		game.Score = 0
		game.Congestion = 10
		game.Parkings["PARK_A"].Occupied = 0
		game.Parkings["PARK_B"].Occupied = 0
	} else if req.Action == "REJECT" {
		if !car.IsApplied {
			game.Score += 100
			msg = "未申請車両を適切に追い返しました！ (+100pt)"
		} else {
			game.Score -= 150
			game.Congestion += 20
			msg = "警告！申請済みの車両を追い返しました！ (渋滞+20)"
		}
	} else {
		lot, exists := game.Parkings[req.Action]
		if exists {
			if lot.Occupied >= lot.Capacity {
				game.Congestion += 25
				msg = lot.Name + "は満車です！(渋滞+25)"
			} else if req.Action == "PARK_A" && (!car.IsApplied || car.VehicleType == "TRUCK") {
				game.Score -= 100
				game.Congestion += 20
				msg = "A駐車場の入構ルール違反です！(渋滞+20)"
			} else {
				lot.Occupied++
				game.Score += 150
				if game.Congestion > 5 {
					game.Congestion -= 5
				}
				msg = fmt.Sprintf("%s へ誘導成功！ (+150pt)", lot.Name)

				pyReq, _ := json.Marshal(map[string]interface{}{
					"start_node":      car.EntryGate,
					"target_parking":  req.Action,
					"congestion_jct1": game.Congestion / 5,
				})
				client := http.Client{Timeout: 5 * time.Second}
				resp, err := client.Post(getPythonEngineURL()+"/route", "application/json", bytes.NewBuffer(pyReq))
				if err == nil {
					defer resp.Body.Close()
					json.NewDecoder(resp.Body).Decode(&routeData)
					if t, ok := routeData["py_exec_ms"].(float64); ok {
						pyTime = t
					}
					game.Metrics.EngineMode = "Python NetworkX Connected"
				} else {
					game.Metrics.EngineMode = "Fallback (Python Offline)"
				}
			}
		}
	}

	game.CurrentCar = generateVehicle()
	game.LastRoute = routeData
	game.Metrics.LastGoTimeMs = float64(time.Since(startGo).Microseconds()) / 1000.0
	game.Metrics.LastPyTimeMs = pyTime
	game.Metrics.LastAction = fmt.Sprintf("[%s] %s -> %s", time.Now().Format("15:04:05"), game.PlayerName, req.Action)

	broadcastState()
	game.mu.Unlock()

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]interface{}{
		"message": msg,
		"state":   game,
		"route":   routeData,
	})
}

func handleMapProxy(w http.ResponseWriter, r *http.Request) {
	client := http.Client{Timeout: 5 * time.Second}
	resp, err := client.Get(getPythonEngineURL() + "/map")
	if err != nil {
		// Python未接続時でも画面が落ちないようにデフォルトマップを返す
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(`{"nodes":{"GATE_MAIN":{"x":80,"y":250,"type":"gate"},"GATE_WEST":{"x":80,"y":80,"type":"gate"},"JCT_1":{"x":260,"y":165,"type":"junction"},"JCT_2":{"x":440,"y":165,"type":"junction"},"PARK_A":{"x":620,"y":80,"type":"parking"},"PARK_B":{"x":620,"y":250,"type":"parking"}},"edges":[{"from":"GATE_MAIN","to":"JCT_1"},{"from":"GATE_WEST","to":"JCT_1"},{"from":"JCT_1","to":"JCT_2"},{"from":"JCT_1","to":"PARK_B"},{"from":"JCT_2","to":"PARK_A"},{"from":"JCT_2","to":"PARK_B"}]}`))
		return
	}
	defer resp.Body.Close()
	w.Header().Set("Content-Type", "application/json")
	io.Copy(w, resp.Body)
}

// 【新規追加】実地図データ (campus_graph.json) を配信するAPI
func handleRealMap(w http.ResponseWriter, r *http.Request) {
	data, err := os.ReadFile("../colab-engine/campus_graph.json")
	if err != nil {
		data, err = os.ReadFile("campus_graph.json")
		if err != nil {
			http.Error(w, "実地図データ(campus_graph.json)が見つかりません", http.StatusInternalServerError)
			log.Println("Error reading real map:", err)
			return
		}
	}
	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Access-Control-Allow-Origin", "*")
	w.Write(data)
}

func main() {
	rand.Seed(time.Now().UnixNano())
	go startAutoExitLoop()

	http.Handle("/", http.FileServer(http.Dir("./static")))
	http.HandleFunc("/api/state", handleState)
	http.HandleFunc("/api/dispatch", handleDispatch)
	http.HandleFunc("/api/map", handleMapProxy)
	http.HandleFunc("/api/real-map", handleRealMap) // 実地図APIを追加
	http.HandleFunc("/api/stream", handleStream)

	fmt.Println("🚀 Go Game Server running at http://localhost:8080/index.html")
	fmt.Println("📊 Dashboard running at http://localhost:8080/dashboard.html")
	fmt.Println("🗺️  Real Map API running at http://localhost:8080/api/real-map")
	fmt.Println("🔗 Python Engine URL:", getPythonEngineURL())
	initTwinEngine()
	log.Fatal(http.ListenAndServe(":8080", nil))
}
