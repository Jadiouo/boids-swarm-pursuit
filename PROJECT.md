# ROS 2 Boids 群體協同追捕 — 專案介紹

一個用 **ROS 2 (Jazzy)** 打造的多機器人群體智慧沙盒:一群 boids(鳥群/群體 agent)透過**分散式、局部感知**的協同,合作圍捕一個速度是自己 **2 倍**的目標。單一 agent 永遠追不上,唯有靠**預測、包抄、包圍、驅趕**等群體戰術才可能得手 —— 這正是專案的核心趣味與研究價值。

最終目標是把這套群體控制邏輯遷移到真實無人機(Crazyflie / Crazyswarm2),所以整個設計刻意保持「分散式 + 局部感知」的骨架。

---

## 三個演進階段

專案依三份 SDD 逐步演進,每一版都建立在前一版之上:

| 版本 | 主題 | 顯示層 | 產出 |
|---|---|---|---|
| **v2** | Boids 群聚核心 | ROS 2 `turtlesim` | 分離/對齊/凝聚/邊界/漫遊 + 非完整約束運動學轉換 |
| **v3** | 協同追捕遊戲 | 自製 **pygame** 世界 | 追捕遊戲迴圈、T0–T4 策略階梯、捕獲/計分、可玩 human 模式 |
| **v4** | 感知模型、進階控制、程序化環境 | 同上 | 真實感測器模型、航跡追蹤、資訊分享、反周界戰術、程序化地圖、自適應目標 |

- **`boids_turtlesim/`** — v2 套件(turtlesim 群聚)
- **`boids_swarm/`** — v3 + v4 主套件(pygame 世界 + 追捕遊戲)

---

## 系統架構(分散式 + 唯一世界擁有者)

```
        ┌─────────────────────────────┐
        │  pygame_sim_node（唯一世界）  │  物理 · 碰撞 · 捕獲 · 渲染 · 計分
        │  合成每-agent 感測 · 發 /clock │
        └──────┬───────────────┬──────┘
     pose/detections     cmd_vel │  ▲ cmd_vel
               ▼               │  │
   ┌───────────────────┐      │  │   ┌────────────────────────┐
   │ boid_controller ×N │──────┘  └───│ target_controller（逃者）│
   │ 群聚+追擊+追蹤+搜尋 │             │ reactive / adaptive 大腦 │
   └───────────────────┘             └────────────────────────┘
```

- **每個 agent 跑自己的 controller 節點**(獨立 process、獨立 namespace `/agent0..N`),沒有中央大腦 —— 這是「分散式」的本質。
- **sim 是唯一的世界狀態擁有者**;controllers 只透過 ROS 主題溝通(pose/detections 進、cmd_vel 出)。
- sim 發布 `/clock`,controllers 用 `use_sim_time`,所以 headless 快轉基準測試仍公平可重現。

---

## 主要功能(v4 現況)

### 感知(可切 `perception:=perfect|sensor`)
- **perfect**:v3 基準,廣播全域真值(回歸用)。
- **sensor**:sim 為每個 agent 合成「牠實際看得到的」—— **視野錐 FOV**、**遮蔽 ray-cast**、**隨距離增長的雜訊**、**偵測丟失**,只發相對 range/bearing。每個 agent 各只訂閱自己的 `/agent{i}/detections`。

### 追蹤 · 搜尋 · 資訊分享
- **航跡濾波**(alpha-beta + 資料關聯):平滑雜訊、橋接丟失、從速度**導出**航向/速度。
- **搜尋**:全員看不到目標時,依 index 扇形散開掃描,直到重新捕獲。
- **資訊分享**(`comms.py`):看到目標的 agent 沿**距離限制的網狀鏈**廣播,群體因此能追蹤大多數個體看不到的目標 —— **「群體握有單一個體沒有的知識」**,個體 vs 群體的分界。

### 追捕戰術(`strategy:=`)
- 基礎階梯:`naive` / `intercept` / `pincer` / `encircle`(先包圍再收攏)/ `herd`
- 反周界(對付貼牆跑者):`counter_rotate` / `blockade`(提前堵路,最強)/ `corner_trap` / `herd_inward`
- 新隊形:`sweep`(貼牆線 cordon)/ `role_encircle`(非對稱收網)/ `bait`(誘餌開口)
- **`auto`(預設)**:每個 boid 依情境自動選策略 —— 目標繞周界→blockade、貼角落→corner_trap、開闊→encircle。
- **終端撲擊**:靠近目標時直接撲上(而非繞圈),果斷收網。

### 程序化環境(`env:=`,種子決定式)
- `obstacle_field`(**固定手工地圖** —— 16 顆大小不一的圓鋪滿整張、彼此留通道,每次都一模一樣;避障另加**切向滑過**分量讓 agent 弧線繞過障礙)、`pillar`(中央柱)、`zones`(綠=捕獲區直接贏、琥珀=焦油坑抵消 2× 速)、`shrink`(競技場邊界內縮,讓周界迴圈物理上不可能)。

### 自適應目標(`evader:=reactive|adaptive`)
- **adaptive**:效用選擇器在行為 repertoire(逃離 / 沿牆跑 / 急閃 / 穿隙 / 障礙掩護)上依威脅幾何評分,含遲滯避免抖動。與自適應追捕者形成軍備競賽。

---

## 怎麼跑

```bash
cd ~/final_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select boids_swarm
source install/setup.bash            # 每個新終端都要

# 純群聚(無目標)
ros2 launch boids_swarm flocking.launch.py num_agents:=12

# 追捕遊戲(自動選策略 + 障礙場)
ros2 launch boids_swarm pursuit.launch.py strategy:=auto env:=obstacle_field trails:=true stamina:=true

# 明顯的線形隊形
ros2 launch boids_swarm pursuit.launch.py strategy:=sweep env:=pillar trails:=true

# 感測模型 + 資訊分享(窄 FOV 最能看出群體共知)
ros2 launch boids_swarm pursuit.launch.py perception:=sensor strategy:=auto

# 你親自當 2× 目標(方向鍵,↑ 衝刺)
ros2 launch boids_swarm pursuit.launch.py game_mode:=human strategy:=blockade

# 存 PNG 逐格檢查(視窗看不清時)
ros2 launch boids_swarm pursuit.launch.py strategy:=auto env:=pillar \
    screenshot_dir:=/tmp/frames screenshot_period:=5.0
```

測試:`python3 -m pytest src/boids_swarm/test/ -q`(90 個純數學單元測試)

---

## 值得一提的工程教訓(踩過的坑)

多機器人 ROS 2 控制迴圈有幾個「靜默殺手」—— 節點在跑、主題有流量,但控制品質被毀:

1. **`rclpy.spin_once` 一次只處理一個 callback**,對 N 條 cmd_vel 會積壓成 ~330ms 致動延遲 → 每個轉向迴圈震盪。改用背景執行緒 executor + QoS depth 1。
2. **N×N pose 訂閱在 Python 到 N≈12 就把 CPU 打爆** → 改用聚合主題(v3);v4 感測模式每 agent 只訂自己那條。
3. **P 轉向控制在 ±π 邊界抖動**(延遲下永遠選不定轉向)→ 加轉向遲滯 + 期望向量 EMA。
4. **感測雜訊/丟失會再度衝擊轉向迴圈** → 靠航跡濾波吸收,raw blip 別直接進行為。
5. **共享信念是雙面刃**:全員收斂同一信念會擠成一團(對周界跑者反而更難圍)—— 正是反周界戰術要解的。

完整開發過程與 13 條坑清單見 [log.md](log.md)。

---

## 檔案地圖

```
final_project/
├── PROJECT.md                    # 本檔(專案總覽)
├── log.md                        # 三階段完整開發日誌 + 坑清單
├── SDD/                          # 三份設計文件 v2/v3/v4
└── ros2_ws/src/
    ├── boids_turtlesim/          # v2 turtlesim 群聚
    └── boids_swarm/              # v3+v4 主套件
        ├── boids_swarm/
        │   ├── pygame_sim_node.py     # 世界:物理/渲染/感測合成/區域/收縮
        │   ├── boid_controller_node.py# 群聚+追擊+追蹤+搜尋(每 agent)
        │   ├── target_controller_node.py # 逃者(reactive/adaptive)
        │   ├── perception.py          # 感測器模型(FOV/遮蔽/雜訊/丟失)
        │   ├── tracking.py            # 航跡濾波+關聯+繞圈偵測
        │   ├── comms.py               # 距離限制網狀信念傳播
        │   ├── world_gen.py           # 種子決定式程序化地圖
        │   ├── game.py                # 捕獲條件+計分
        │   ├── geometry.py            # 向量/角度/運動學轉換
        │   └── behaviors/
        │       ├── flocking.py        # 分離/對齊/凝聚/邊界/漫遊/搜尋
        │       ├── pursuit.py         # 全部追捕策略 + auto + 終端撲擊
        │       └── evasion.py         # ReactiveEvader / AdaptiveEvader
        ├── launch/                # flocking / pursuit launch
        ├── config/params.yaml     # 所有可調參數(runtime 可改)
        └── test/                  # 90 個單元測試
```

詳細指令與參數見套件的 [README](ros2_ws/src/boids_swarm/README.md)。
