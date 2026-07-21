# 開發日誌 (log.md)
## ROS 2 Boids Swarm — turtlesim 群聚 (SDD v2) → pygame 協同追擊遊戲 (SDD v3)

> 環境:WSL2 + ROS 2 Jazzy + Python 3.12,12 核。
> Workspace:`~/final_project/ros2_ws`,兩個套件:`boids_turtlesim`(v2)、`boids_swarm`(v3)。

---

## Phase 1 — SDD v2:turtlesim Boids 群聚

### 開發

1. 建立 `boids_turtlesim`(ament_python):
   - `behaviors.py` — 五種行為的純數學(Separation `1/d²`、Alignment 單位向量平均、Cohesion、Boundary 軟牆、Wander)+ §4.5 非完整約束轉換(`forward = max(0, cos e)` 不倒車 + `v_min` 地板)。刻意不含 ROS import,方便單元測試。
   - `boid_controller.py` — pose callback 只寫 cache,固定頻率 timer 跑控制迴圈(§4.1);`add_on_set_parameters_callback` 支援即時調參,含 `control_rate_hz` 動態重建 timer。
   - `swarm_spawner.py` — kill 預設 turtle1、在邊界 margin 內隨機生成 N 隻。
   - launch + `/**` 萬用字元參數 yaml + pytest(19 個測試,涵蓋 SDD 點名的 179°/−179° 角度平均陷阱、不倒車、v_min 地板)。

### 驗證(§8 全過)

- 6 隻、32 秒實跑:最小間距 0.298(無碰撞)、無出界、bbox 穩定 3–4、頭向變異數 < 0.05。
- `cmd_vel` 實測 20.02 Hz;`ros2 param set control_rate_hz 30` 後實測 29.998 Hz 即時生效。

**Phase 1 一次到位,沒有重大坑。**(這句後來很諷刺 —— v3 的坑全在 ROS 佈線層,v2 規模小到沒踩到。)

---

## Phase 2 — SDD v3:pygame 世界 + 協同追擊

### M0–M1:骨架與世界(順利)

- `pygame_sim_node` 成為世界擁有者:固定步長物理(dt=1/fps,可重現)、非完整/完整約束雙模式、圓形障礙物推擠、捕獲判定、HUD、`/clock` 發布(controllers 用 `use_sim_time`,headless 快轉時基準才公平)。
- M1 驗收:發 `cmd_vel` → pose 正確積分移動 ✓。
- 額外做了 `screenshot_dir`/`screenshot_period` 參數(runtime 可開),事後證明是**最重要的除錯工具**。

### M2:群聚 —— 三個接連的 ROS 佈線殺手

離線純數學 mini-sim(同權重、無 ROS)完美收斂(hvar 0.76→0.01),但 ROS 系統死活不群聚。依序抓出三個 bug:

| # | 症狀 | 根因 | 修法 |
|---|------|------|------|
| 1 | 12 個 controller 各吃 70–100% CPU,控制根據過期資料 | **O(N²) pose 訂閱**:N 個 controller × N 條 60Hz pose 流,Python callback 淹沒(v2 SDD §10.2 警告過的問題) | sim 發布聚合 `/swarm/poses`(`Float32MultiArray`,30Hz),每個 controller 只訂 1 個主題 |
| 2 | 修完 #1 CPU 正常,群體會聚攏(bbox 157→27)但頭向永遠混亂、貼身碰撞 | agent 卡在群內平衡點:\|V\|≈0 時 `atan2` 是雜訊;加上延遲,P 控制器在 **±π wrap 邊界抖動**(ω 在 ±max 之間翻轉、原地打轉) | `to_twist` 加轉向遲滯(\|e\|>2.6 鎖定前次轉向)、期望向量 EMA 濾波、own-heading 以 ω·age 外插、deadband 時**原地轉向對齊鄰居**(不能凍結,否則 alignment 共識永遠不形成)、`v_min` 歸零 |
| 3 | 修完 #2 還是不行 —— 最後的真兇 | **`rclpy.spin_once()` 一次只處理一個 callback**。SDD §1.5 偽代碼的「drain cmd_vel」根本 drain 不了:13 條 30Hz cmd 流 vs 每幀 1 個 callback → QoS 佇列(深度10)常滿 → **~330ms 致動延遲**,任何轉向迴圈都會震盪 | executor 移到背景執行緒(callback 只做原子 tuple 寫入,pygame 留在主執行緒);cmd_vel 佇列深度改 1(只留最新) |

修完後教科書級群聚:min 間距 0.7–1.4、hvar → 0.03–0.08、bbox 184→17。

> **方法論教訓**:先寫「無 ROS 的離線 mini-sim」對照 —— 立刻把「數學錯」和「佈線錯」分開;再用 sim 內建截圖直接看行為,比盯指標猜快得多。已存入長期記憶(`ros2-python-control-loop-gotchas`)。

### M3:遊戲迴圈(順利)

- ReactiveEvader(取樣 24 方向評分:追兵威脅 + 牆/障礙淨空 + 轉向代價,天然含「往最大缺口逃」)。
- 回合制:capture/timeout → banner → 自動重生,`EPISODE`/`SUMMARY` 記錄行輸出供基準解析;`episodes_max` 跑完 sim 自動退出(launch `on_exit=Shutdown`)。
- 實跑:目標沿牆逃、群體協同掃向它,ep2 開場 4 秒 close 2/3 —— 迴圈 end-to-end ✓。

### M4–M5:策略基準 —— 遊戲平衡的三輪迭代

基準方法:同種子、headless **有界** time_scale(無上限快轉會再次把 controller 打爆 —— 又一個教訓)、每策略 6 回合。

- **第 1 輪**:全策略幾乎全 timeout,僅有的捕獲都在開場 5–6 秒(出生點運氣)。
  → 診斷:`target_omega_max 2.5` 在 4u/s 下轉彎半徑僅 1.6 —— 目標「又快又靈」,SDD §4.2 明說這樣抓不到。調成 1.2(半徑 3.3)。
- **第 2 輪**:更糟(0 捕獲)。看截圖:目標不知疲倦沿牆繞圈,群體以一半速度永遠追在後面 —— **開放場地的不倦 2× 目標在結構上不可捕**。
  → 啟用 SDD 自己的均衡器:**stamina**(衝刺耗盡→巡航回血),evader 改成「追兵 < panic_distance 才衝刺」;`d_capture` 1.5→2.0。
- **第 3 輪**:還是只有開場捕獲。最後的結構性癥結:**cohesion 讓 12 隻永遠是一個 blob**,以巡航速度追平目標,包圍角張不開。
  → encircle 改「**先包圍再收攏**」:環半徑隨自身距離縮放(`max(schedule, 0.45·dist)`)、slot 改世界座標系;追擊模式 cohesion/alignment 降到 0.2。

最終結果(達到 SDD「possible-but-hard」標準):

| 策略 | 捕獲率 | 備註 |
|---|---|---|
| T0 naive | 1/6 | 只有出生點運氣 —— 正好證明 2× 問題(SDD 預期) |
| T1 intercept | 2/6 | avg 7.4s |
| T4 herd | 2/6 | avg 7.2s |
| T3 encircle | 2/6 @90s;**3/4 @180s** | 持久戰靠 stamina 消耗循環 |

### M6:障礙物 + human 模式

- `obstacles:="x,y,r;..."` launch 參數 → sim 阻擋 + 渲染,boids 有 `V_obs` 排斥項。
- `game_mode:=human`:不生 AI evader、方向鍵駕駛(↑ 衝刺)、HUD 提示。
- 截圖驗證:三個障礙物正確、12 隻 boids 對(閒置的)人類目標形成均勻包圍環 —— T3 視覺成果。實際鍵盤遊玩留給使用者。

### M7(stretch,未做)

`Nav2Evader` 留 stub + §6.4 接線契約(odom/TF/OccupancyGrid/escape-goal loop);Nav2 套件已確認安裝(34 個 nav2_*)。

---

## 最終狀態

- **測試**:40 passed(v2 19 + v3 21),涵蓋角度平均、不倒車、遲滯、捕獲幾何(hull/escape_blocked/tag)、策略幾何(lead、pincer 角色分裂、encircle slot、herd 開放側)。
- **可跑指令**:見 [boids_swarm/README.md](ros2_ws/src/boids_swarm/README.md)。
- **調參入口**:全部參數 runtime 可調(`ros2 param set`),預設值在 `config/params.yaml`。

## 坑清單(速查)

1. `rclpy.spin_once` ≠ drain;手寫迴圈的節點請把 executor 放背景執行緒,「只要最新值」的主題用 QoS depth 1。
2. N×N pose 訂閱在 Python 到 N≈12 就飽和;用聚合陣列主題。
3. P 轉向控制 + 感測延遲會在 ±π 抖動;要加轉向遲滯。
4. deadband 不能凍結 heading,否則 alignment 共識死鎖。
5. headless 快轉要**有界**(time_scale),否則 controller 追不上 sim time;用 `/clock` + `use_sim_time` 保持公平。
6. 平衡調不動時先看行為(截圖/軌跡),不要瞎調權重;「不倦的 2× 目標在開放場地不可捕」是數學,不是調參問題。
7. 離線 no-ROS mini-sim 是分離「數學 bug vs 佈線 bug」最快的工具。

---

## Phase 3 — SDD v4:感測模型、進階協同控制、程序化環境(M8–M14)

v4 針對 v3 兩個實測發現:(1) 不倦的 2× 目標靠貼周界跑成「數學上安全」的 1D 迴圈;(2) 感測不真實(所有 agent 都拿到全域真值)。全部功能都藏在開關後(`perception_mode`/`env`/`evader`/`comms_enabled`),v3 基準維持可比。

### M8 — 感測模型(`perception.py`)

sim(唯一知道真值者)為每個 agent 合成「牠實際偵測到的東西」:FOV 錐、遮蔽 ray-cast、隨距離增長的雜訊、丟失機率,只發相對 range/bearing 到 `/agent{i}/detections`。`perfect` 模式保留 v3 廣播當回歸基準。
- 驗證:omni FOV 每 agent K≈5–7 → 窄錐(0.5 rad)K=0–3(agent6 全盲);perfect 回歸 v3。

### M9 — 追蹤 + 搜尋(`tracking.py`)

alpha-beta 濾波 + 資料關聯(有 id 用 id、無 id 用最近鄰),從航跡速度**導出**航向/速度(range/bearing 感測器本來給不了),餵給行為。目標丟失時做覆蓋式搜尋(每 agent 依 index 掃不同扇區)。
- 驗證:sensor 模式群聚在雜訊+丟失下仍穩定(min 距 0.77 vs perfect 0.55,hvar 0.22 vs 0.12,無碰撞);窄 FOV 時群體扇形散開搜尋、重新捕獲。

### M10 — 資訊分享(`comms.py`)

sim 建 comm graph(距離 < `comm_range` 連邊),用 union-find 把「看到目標」的信念沿連通分量傳播。看到目標的 agent 廣播,其他中繼。
- 驗證(關鍵是機制,非捕獲率):窄 FOV 下 comms 關 → 平均 **2.3/12** agent 持有目標信念(僅直接看到者);comms 開 → **12/12**,每一幀。這就是「群體握有單一個體沒有的知識」——個體 vs 群體的分界首次出現。

### M11 — 反周界戰術(`pursuit.py` + 繞圈偵測)

`CirclingDetector`(在共享航跡上算對競技場中心的角速度,持續+貼牆才算繞圈)。四個新策略:
- **counter_rotate**:半群反向繞同一環,閉環上保證迎面攔截。
- **blockade**:偵測繞圈後外插目標軌道遠端,派 1–2 隻**提前到位並定點堵住**(對靜止塞子,2× 速度無意義)。
- **corner_trap / herd_inward**:預判下個角落預置 / 從牆側把目標推回中央。
- 驗證(同種子 11/23/31,對貼周界目標):**blockade 7/9** vs encircle(v3 最佳)**3/9** —— 明顯勝出。counter_rotate/corner_trap 3/9 打平。

### M12 — 新隊形

- **sweep**:貼牆兩端的橫線 cordon,往目標最近的牆推,壓縮可達區(縮空間,非圍點)。
- **role_encircle**:目標逃向側的 boid 撐大半徑當塞子、後方 boid 收小 —— 非對稱收網。
- **bait**:故意在環上留一條通往角落 kill-box 的缺口。
- 驗證:三者實跑無崩潰,bait 6.2s 捕獲 1/1。

### M13 — 程序化環境(`world_gen.py`)

種子決定式 `WorldGenerator`:`obstacle_field`(Poisson 散佈 + 貼牆塞子斷周界)、`pillar`、`zones`(綠=捕獲區直接贏、琥珀=焦油坑抵消 2× 速)、`shrink`(競技場邊界隨時間內縮,周界迴圈短於轉彎半徑 → 繞圈物理上不可能)。sim 加了收縮邊界(發 `/arena/bounds`,controllers 即時更新邊界避讓)、區域效果、區域渲染。
- 驗證:zones 正確渲染(綠/琥珀)+ 捕獲區入區判定;shrink 把 20×20 擠到 4×4,捕獲 3/4(殘留 1 次逾時是共享信念下群體擠成一團、貼牆目標未被完整包圍 —— 是戰術分佈問題,非收縮失效)。

### M14 — 自適應目標(`evasion.py`)

`AdaptiveEvader`:效用選擇器在行為 repertoire(retreat / 沿牆跑 / juke 急閃 / gap_dash 穿隙 / obstacle_shield)上依威脅幾何(最近追兵、包圍完整度、離牆/障礙距離、體力)評分取 argmax,含遲滯避免抖動。可解釋加權啟發式,非學習策略。
- 驗證:實跑切換 retreat→perimeter→juke→gap_dash,模式切換有 log 且對應威脅幾何(追兵近→juke、被圍→gap_dash)。

### v4 坑清單(接續 v3)

8. 感測雜訊/丟失會再度衝擊 P 轉向迴圈(疊加 v3 坑 #3)—— 靠 `tracking.py` 濾波吸收;raw blip 千萬別直接進行為。
9. 每 agent 一個偵測主題,但**各只訂自己那條**(比 v3 廣播訂閱更少),維持 v3 的背景執行緒 executor + QoS depth 1。
10. 戰術偵測器(繞圈、下個角落)依賴共享航跡品質 —— 要在 `sensor` 模式驗證,不能只看 `perfect`。
11. 程序化佈局必須種子決定式,否則策略基準變雜訊(在 launch 端一次生成、同時發給 sim 與 controllers,避免雙重生成不一致)。
12. 收縮競技場要靠「發 `/arena/bounds` 讓 controllers 即時更新邊界」,v2 §4.3.4 邊界避讓才會跟著動牆;光靠 sim 夾位置、boids 仍以為牆在 20 會擠爆。
13. 共享信念是雙面刃:所有 boid 收斂到同一信念會擠成一團(對周界跑者反而更難圍)—— 這正是 M11/M12 戰術要解的問題,不是 comms 的 bug。

### 最終狀態(v4)

- **測試**:85 passed(涵蓋 flocking、捕獲幾何、v3+M11+M12 策略、感測 FOV/遮蔽/雜訊/丟失/往返、追蹤 alpha-beta/關聯/繞圈、comms mesh、程序化生成、自適應 evader)。
- **新增模組**:`perception.py`、`tracking.py`、`comms.py`、`world_gen.py`;`behaviors/pursuit.py`(+7 策略)、`behaviors/evasion.py`(+AdaptiveEvader)。
- **主要開關**:`perception:=perfect|sensor`、`env:=open|obstacle_field|pillar|zones|shrink`、`evader:=reactive|adaptive`、`comms_enabled`、各感測元件可獨立 ablate。
- **剩餘 stretch**:M7/E.4 `Nav2Evader`(契約已備,Nav2 已裝);v4 的自適應 retreat-to-open 正是給 Nav2 的真實用途。
