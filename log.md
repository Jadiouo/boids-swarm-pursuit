# 開發日誌 (log.md)
## ROS 2 Boids Swarm — turtlesim 群聚 (SDD v2) → pygame 協同追擊遊戲 (SDD v3)

> 環境:WSL2 + ROS 2 Jazzy + Python 3.12,12 核。
> Workspace:`~/boids-swarm-pursuit/ros2_ws`,兩個套件:`boids_turtlesim`(v2)、`boids_swarm`(v3)。

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

種子決定式 `WorldGenerator`:`obstacle_field`(**最後改成固定手工地圖** —— 原本是 Poisson 散佈 + 貼牆塞子,但每個 seed 的難度差太多,策略基準不可比,所以改成 12 顆手工擺位的圓、每次都一樣)、`pillar`、`zones`(綠=捕獲區直接贏、琥珀=焦油坑抵消 2× 速)、`shrink`(競技場邊界隨時間內縮,周界迴圈短於轉彎半徑 → 繞圈物理上不可能)。sim 加了收縮邊界(發 `/arena/bounds`,controllers 即時更新邊界避讓)、區域效果、區域渲染。
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

---

## Phase 4 — 收尾(2026-07-28)

功能凍結,只做「讓別人 clone 下來能跑、文件不說謊」。**沒有動任何演算法權重**,唯一的行為變更是 `_shield_dir`(下方 #3),已補三個單元測試。

### 可跑性(乾淨 clone 撞到的)

1. **pygame 沒寫在安裝步驟** —— `package.xml` 有 `<exec_depend>python3-pygame</exec_depend>`,但 README 的四步 build 沒提,照做完 `pygame_sim` 直接 `ModuleNotFoundError`。三份 README 都補上 `apt install python3-pygame` / `rosdep install`。
2. **文件的測試指令跑不起來** —— `python3 -m pytest src/boids_swarm/test/ -q` 在沒 `colcon build` + source 的情況下 9 個 collection error(`No module named 'boids_swarm'`)。加 `ros2_ws/pytest.ini` 設 `pythonpath`,現在 `cd ros2_ws && python3 -m pytest -q` 直接可跑,**且不需要 ROS 也不需要 pygame**(測試本來就是純數學)。
3. 所有文件路徑 `~/final_project/ros2_ws` → `~/boids-swarm-pursuit/ros2_ws`。
4. 補 `LICENSE`(MIT,`package.xml`/`setup.py` 本來就這樣宣告,只是檔案沒有)。

### 文件與程式對不上

5. 「90 個單元測試」→ 實際 **114**(修完為止)。
6. `README.md` 與 `PROJECT.md` **位元組完全相同** → `PROJECT.md` 改成指向 `README.md` 的 symlink。
7. 招牌寫「2× 速度」,但 `params.yaml` 出貨 `target_speed_multiplier: 1.8`;`target_body_radius` 出貨 0.15 讓 SDD C.1「大目標繞路」實際沒啟用。**保留現值**(那是實測平衡),改成在 README 和 params.yaml 兩邊都寫清楚「規格上限 vs 出貨值」。
8. `obstacle_field`「16 顆圓」→ 實際 `FIXED_FIELD` 12 顆。
9. 本檔 M13 還寫 Poisson 散佈 → 已補上「後來改固定地圖」的原因。
10. 套件 README 有兩個 `## Tests` 章節 → 合併。
11. `setup.py`/`package.xml` 還寫 `0.3.0` / 「SDD v3」→ `0.4.0` / 「SDD v3+v4」。

### 程式碼

12. **`AdaptiveEvader.compute(..., stamina=1.0)` 從來沒被傳過** —— `target_controller_node` 只給三個引數,`self._stamina` 寫入後從未被讀,docstring 卻宣稱評分含 stamina。sim 才是 stamina 的擁有者,要接需要新的 topic;所以移掉死參數,docstring 改成明說「stamina 不是輸入,衝刺由 `panic_distance` 控制」。
13. **`_shield_dir` 名不符實** —— docstring 說「把障礙物擋在自己和追兵之間」,實作只是朝**最近的**障礙物走,完全沒用追兵方向,可能一頭撞向追兵那側。改成只採用位於追兵反側的障礙物(取最近的),沒有合格的就回 `None` 讓選擇器捨棄這個選項。補 3 個測試。
14. **`episodes_max` 配 `auto_reset:=false` 永遠不會結束** —— 舊碼只有在 auto_reset 分支才會遞增 episode,`episode > em` 因此永不成立,sim 卡在 banner。改成兩種結束條件都會收斂,`_final_summary` 的 `episodes` 也跟著從 `episode - 1` 改成 `episode`(現在 episode 就是實跑數)。
15. **策略名打錯會靜默 fallback 到 `intercept`** —— 跑基準 sweep 時這會把「沒跑到的策略」記成有效數據。改成第一次遇到就 `warn` 並列出合法值。
16. **`evader:=` 打錯直接 `KeyError` crash** → 改成清楚的 `RuntimeError` 訊息。
17. **launch 寫死 `WorldGenerator(seed, 20.0)`** —— `params.yaml` 的 `world_size` 改了,程序化地圖不會跟著縮放。改成從 params.yaml 讀,讀不到才退回 20.0。
18. 死碼清除:`WorldGenerator._scatter`(改固定地圖後沒人叫)、`Scoreboard.boids_lost`、`pursuit.corner_trap` 的 `aim`、`comms.py` 的 `import math`、`target_controller_node` 的 `n`、3 處測試未用符號。`WorldGenerator` 的 `__import__('random')` 改成正常 import。**pyflakes 全乾淨**。
19. `SDD/ Sdd_boids_v4.md` 檔名開頭有一個空格 → 移除。
20. 加 GitHub Actions:每次 push 跑 114 個測試 + pyflakes。

### 收尾後的實跑驗證(全新環境,ROS 2 Jazzy + pygame 2.6.1)

先 `colcon build --symlink-install` 兩個套件(只有 setuptools deprecation 警告),然後:

| 驗的東西 | 指令 | 結果 |
|---|---|---|
| 追捕遊戲 end-to-end | `headless episodes_max:=4 seed:=11 strategy:=blockade time_limit:=45 N=8` | 3 timeout + 1 capture(6.88s),`SUMMARY episodes=4` **計數正確**、rc=0 乾淨退出 |
| #15 策略名警告 | `strategy:=encirlce`(故意打錯) | 每個 agent 都 `WARN unknown pursuit_strategy 'encirlce' — falling back to 'intercept'`,並列出 13 個合法值 |
| #14 `auto_reset:=false` | 單跑 `pygame_sim -p auto_reset:=false -p episode_time_limit:=5` | **2 秒內自己結束**並印 `SUMMARY episodes=1`(修之前會永遠卡在 banner) |
| #8 地圖顆數 / #17 world_size | `env:=obstacle_field` | sim 回報 `obstacles=12`(不是文件原本寫的 16);截圖目視 12 顆、通道連通 |
| #13 `_shield_dir` | `evader:=adaptive env:=obstacle_field` | 五種模式 retreat/perimeter/juke/gap_dash/**shield** 全部觸發,無例外 |
| 感測鏈路 | `perception:=sensor strategy:=auto` | 兩回合跑完無 exception(感測合成→追蹤→comms→行為 全通) |
| 程序化環境 | `env:=shrink` / `zones` / `pillar` | shrink 8.78s 捕獲、zones 9.35s 捕獲、pillar timeout;皆無崩潰 |
| v2 群聚回歸 | `flocking.launch.py num_agents:=12` | 12 隻同向等距、平行軌跡、`min_d 0.54` 無碰撞 —— 與 M2 驗收指標一致 |

### 兩個沒修、但記下來的觀察

- **`SUMMARY ... avg_capture_t=nans`** —— 零捕獲時 `float('nan')` 套 `{:.2f}s` 印成 `nans`。難看,但解析器照樣 `strip('s')` → `float('nan')` 可解,改格式反而會弄壞既有的 log 解析,所以不動。
- **`AdaptiveEvader` 的遲滯擋不住抖動** —— docstring 說「含遲滯避免抖動」,但 30 Hz 控制率下實測模式切換到約 **10 Hz**(343.157→343.181→343.188→343.304 秒各換一次)。`SWITCH_MARGIN = 0.15` 對效用值的尺度來說太小。這是調參問題不是壞掉,修它會改變 evader 行為,留給下一輪。

---

## Phase 5 — M15:視窗內控制面板

目標:把「可調的策略與模式」變成滑鼠可操作的 GUI,和 pygame 世界同一個視窗。

### 做法

- **`ui.py`** —— immediate-mode widget(Section / Cycler / Toggle / Slider / Button / ReadOnly)。**面板不持有任何數值**:每幀跟 sim 要參數現值再畫,所以滑鼠改和 `ros2 param set` 改永遠一致,只有一個真相來源。刻意不在 module scope import pygame(延續 sim 的 lazy import 風格),版面/命中/數值映射因此是純數學、CI 沒 pygame 也測得動。
- **`param_bridge.py`** —— `pursuit_strategy` 其實住在 **N 個獨立 controller 行程**上,面板改一次 = N+1 個 `SetParameters` service call。**執行緒契約**:pygame 在主執行緒、rclpy callback 在背景 executor 執行緒,所以點擊只負責 `put` 進佇列,由一個 ROS timer(executor 執行緒)排空並發送 —— 全部 rclpy 工作維持單執行緒。請求依 (scope, name) **合併**:拖一次滑桿會產生上百個值,只有最後一個有意義。
- **sim 端鏡像參數** —— 面板要畫 `pursuit_strategy` 的現值,但它不在 sim 上。每幀去問 N 個遠端節點太荒謬,所以 sim 宣告一份本地鏡像(params.yaml 的 `/**` 萬用字元讓它自動拿到同樣預設),`ParamBridge` 每次改動同時寫兩邊,launch 也把 `strategy`/`evader` 一併轉發給 sim。
- **`evader` 變成可熱切換** —— brain 物件不持有 ROS 狀態,param callback 直接重建即可;重建時沿用 `brain.bmin/bmax`,避免在收縮競技場中途換腦把牆的模型重設。
- **`target_controller` 不再快取 `v_max`/`w_max`** —— 原本在 `__init__` 算完就存起來,面板改速度後 evader 會照著世界已經不允許的速度轉向。改成每個 cycle 現讀。
- **PAUSE 凍結 `/clock`** —— 暫停時不推進 sim_time 但**繼續發布**,controllers 的 sim-time timer 跟著凍結,而不是對著過期 pose 空轉直到 `pose_timeout`。
- **RESET EPISODE 用新的 `Scoreboard.restart_episode()`** —— 重置計時與 per-episode 指標但**不遞增 episode**,手動重開不會灌水捕獲率的分母。

### 驗證

- 單元測試 +21(`test_ui.py`):循環兩端繞回、未知現值不炸、滑桿端點/量化/夾範圍、Button 不索取數值、拖曳離開列仍跟隨、出貨版面無重複控制項且高度 748px 塞得進 800。
- **端到端扇出實測**(乾淨環境,8 agents):`intercept → herd` 在 agent0/3/7 全部生效;float `w_pursuit=3.7`、bool `search_enabled=False`、字串 `evader=adaptive` 三種型別都正確;連送 4 個 `commit_distance` 只有最後的 4.4 落地(合併有效);`target_controller` 印出 `evader brain -> adaptive`。
- 截圖確認面板渲染:1080×800 視窗、所有控制項數值與 launch 參數一致。

### M15 抓到的兩個 bug

21. **`Toggle` 的建構子參數順序和其他 widget 相反** —— 它沿用基底的 `(label, key, scope)`,但我照 `(key, label, scope)` 呼叫,結果「軌跡」開關會去設一個不存在的參數名 `trails`(真名是 `render_trails`),執行期靜默失敗。單元測試當場抓到。所有帶值的 widget 現在都強制 `(key, label, scope)`。
22. **`rcl_interfaces` 從來沒宣告在 `package.xml`** —— 程式從 v2 就在 `from rcl_interfaces.msg import SetParametersResult`,但它只是碰巧被 rclpy 拉進來。M15 開始用 `SetParameters` **service** 後補上宣告。

### ⚠️ 收尾階段我自己弄壞的東西(GUI 反而抓到)

第一張 GUI 截圖顯示 `turn rate 2.50`,但 params.yaml 應該是 **1.2**。追下去發現:**Phase 4 幫 `target_speed_multiplier` 加註解時,整行 `target_omega_max: 1.2` 被我一起刪掉了**,sim 因此退回自己的宣告預設 2.5。

而這正是 M4/M5 記載的關鍵平衡旋鈕 —— 2.5 在衝刺速度下轉彎半徑僅 1.6,「又快又靈」,SDD §4.2 明說這樣抓不到。

代價很具體:Phase 4 那一輪「blockade seed=11 只有 1/4」的驗證數字**是壞的**。修回 1.2 後同種子重跑 → **2/4**(7.25s、7.52s)。

三個教訓:

- **改 YAML 註解等於改設定** —— 我以為在加說明,實際刪掉了一行值。整段替換式的編輯,事後一定要用「解析後 diff key 集合」比對,不能只看文字 diff 看起來合理。修完立刻加了這個檢查,確認其餘參數一個沒少。
- **靜默退回宣告預設是第二個「靜默 fallback」毒藥** —— 和坑 #16 同一類:params.yaml 少一行,系統照跑不誤,只是變弱。ROS 參數沒有「必填」的概念,所以關鍵旋鈕要在 YAML 裡寫明「不要刪這行」。
- **把狀態畫出來就會被發現** —— 這個迴歸躲過了 114 個單元測試、pyflakes、和八次實跑基準,卻在 GUI 第一張截圖就露餡,因為面板逼著系統把 `target_omega_max` 顯示出來。**可觀察性本身就是測試。**

### 收尾坑清單(接續)

14. **「文件寫的指令」和「你平常打的指令」會分岔** —— 你的 shell 早就 source 過 install/setup.bash,所以文件裡少一步安裝、少一個 PYTHONPATH 你永遠不會發現。收尾時一定要用**乾淨環境**照著自己的 README 走一遍。
15. **調參讓遊戲好玩,會讓文件變成謊話** —— 1.8× / body_radius 0.15 都是為了可玩性調的,但招牌還掛著 2× 和「大目標繞路」。調參當下就該回頭改文件,不然半年後沒人知道哪個才算數。
16. **靜默 fallback 是基準測試的毒藥** —— `dict.get(name, default)` 在設定檔驅動的實驗裡等於偽造數據。不合法的值要嘛吵、要嘛炸,不能安靜。


---

## Phase 6 — 一跳 ROS 觀測共享與驗證（2026-10-07）

**停點：**Phase 1 一跳 ROS relay、文件及小型實驗已驗收；捕獲效能改善未開始。無本專案程序執行。工作樹有未提交檔案；本次只寫日誌，無新實驗。

**方向與交付：**選 B「真 ROS 一跳量測共享」為核心、A「固定條件小實驗」作證據；C「延遲／丟包、事件回放、大 sweep」延後。紅方指出舊 relay 使用 simulator 真值，DDS 全域可達不等於無線網路；白方建議先守 freshness、reset、direct priority、失聯搜尋。range gate 仍依 simulator pose，在應用層限制一跳。完成訊息、權威 epoch/seq、驗證仲裁、原始 TTL、獨立 tracker、四模式相容、telemetry/runner；ROS mode 排除舊 array target，不轉送 relay track。方向與設計見[方向](docs/planning/portfolio-direction.md)、[SDD](SDD/Sdd_distributed_sighting_relay_v1.md)、[TDD](docs/testing/distributed-sighting-relay-tdd.md)。

**修正與驗收：**修正測試歷史 `any` 假陽性（改按 agent/phase/stamp 篩選、drain fixture）、neighbor callback 重融 target／續 TTL、手動 reset epoch generation、QoS、launch mode 宣告及 msg package build type。上一輪驗收：純測試 **156 passed**、真 ROS 雙節點 **2 passed**、pyflakes/diff-check 通過；Jazzy build、perfect 和 sensor/legacy launch 均輸出 SUMMARY。本次沒有重跑。

**實驗與限制：**9 base runs（3 modes × seeds 11/23/37）加 3 個 seed 11 repeats，共 12 次；全數在模擬 30.02 秒 timeout，無捕獲，不能聲稱效能改善或策略勝出。4 個 ROS runs 的 collector 共同 local/shared key 全欄位一致為 1015/1015、1139/1139、1102/1102、955/955；BEST_EFFORT KEEP_LAST(1) 漏收部分 controller 已接受 key，不能稱全網 delivery 100%。coverage 是 receiver 機會覆蓋，非 packet ratio；age 是 simulation time，非 DDS wall latency；range gate 依 simulator pose。詳見[實驗證據](artifacts/sighting-relay-final-2026-10-07/README.md)。

Source hash `b7f28c058e87b00ce14d0c6b192832c458e02c5536598a5abc517f6c26204acc`；[41 檔 snapshot](artifacts/sighting-relay-final-2026-10-07/source_snapshot.tar.gz) 84 KB，evidence 約 40.7 MB；本日誌不在 fingerprint 範圍。

**建議，尚未執行：**以固定少量 seed/場景查 sighting 失聯、重獲及攔截失敗，分辨控制、感知、通訊因素後再定改善與指標；之後才評估回放或延遲／丟包。目前無實驗執行，不預先承諾新功能。


---

## Phase 7 — M7 Nav2 evader（2026-10-08）

**交付：**`evader:=nav2` 可用：逃跑目標取樣評分 → ComputePathToPose/FollowPath → 與 ReactiveEvader 依最近 boid 距離線性混合，`/target/evader_status` 可觀察 mode（nav2/blend/reactive）與失敗計數；障礙由 launch 單一來源同步給 sim／controller／bridge；控制器限速由 params.yaml 推導（3.6 m/s、1.2 rad/s）。細節見 `ros2_ws/src/boids_swarm/README.md`「Nav2 evader (M7)」。

**調參結論（原版，已被下方〈紅方修正〉更正）：**MPPI 嘗試 7 組，但 v3 harness 下有效的只有 4 組（m00/m04/m05/m07）皆未達標，另 3 組只在不可比的 v1/v2 harness 跑過；RPP 勝出的 planner 半徑修正當時沒有套給 MPPI；RPP（曲率調節半徑 = v_max/ω_max）在開闊與中等障礙場 cruise 2.5–3.2、無碰撞無停住，密集迷宮 2.2、貼牆起步 1.9 未達 2.5。全部嘗試含失敗組合在 `docs/testing/nav2-mppi-tuning.md`。

**實跑才發現的坑：**貼牆起步 navfn 拒絕規劃（規劃半徑 > 身體半徑）、RPP collision check 在牆邊永遠 abort、nav2 target 進程比 boid 啟動慢造成追捕者起跑領先（用 `warmup`／`pursuer_delay` 處理）、牆與大障礙之間 0.6 m 的口袋會讓目標卡住（降低、未根除）。

**限制：**sanity 基準（`artifacts/nav2-m7-sanity/`）每組 5 runs、同 seed 重跑結果會翻轉，**不能**據此說 nav2 比 reactive 強或弱；正式比較須另行預先登記。

### 紅方修正（2026-10-08，同日）

**核實的問題：**口袋自困是參數不一致，不是運氣：planner footprint（0.22）> 身體（0.15）、牆畫在競技場內側、plan_start 內縮（0.3）< 牆+半徑。另有失敗後同一不可達 goal 反覆重送、逾時不 cancel 造成晚到結果重複處理、逃跑點只看歐氏距離、`use_collision_detection: false` 使 local costmap 對行為無影響、`(v, w)` 線性混合讓反向轉向相消、非 panic 時速度被夾在 2.0（stamina 設計，非 bug）。

**修了什麼：**牆改畫在競技場外側（貼牆格 cost≈203，高但可通行）、global/local `robot_radius` 由 `target_body_radius` 在 launch 推導；失敗 goal 進黑名單（時間到期、重複加倍）、fallback_hold 期間不重送、逾時 cancel 並以 seq 擋晚到結果；`EscapeMap` 做 grid 測地距離（不可達淘汰、目標先到的 lead、死巷懲罰，每次決策約 13 ms）；blend 改在航向空間。整合測試補了 boid 於 local+global costmap（即時取樣、清空後負對照）、口袋起點、回退與 preempt。

**重新量測（非預先登記）：**plan 失敗率 45/103 → 3/61 與 20/100（同程式兩次重跑差很多）；時間加權 mode：純 nav2 只占 9–22 %；目標在遊戲內平均速度 nav2 1.9–2.3 vs reactive 2.7–2.8 m/s。RPP collision detection 重開後迷宮與貼牆起步都 abort，維持關閉。迷宮 cruise 2.20→2.46、鑽石 2.52→2.35（迷宮有一條 chain 貼到障礙 0.01 m）。**沒有**宣稱 nav2 優於或劣於 reactive。

**仍未達成：**Nav2 控制器本身不躲 boids；sanity seed 6 仍有 15 次 plan 失敗（目標被卡在牆邊口袋約 23 s，推論與 boid 堵住出口有關）；「開闊」與「避免自困」只有單元測試層級的證據，沒有隔離的實驗；MPPI 只在兩個場景重測。

**教訓：**(1) 整合測試要驗「資料來源」——先前 cost>0 測試在 boid 還沒更新時取位置、也沒有負對照；加上負對照才發現 obstacle layer 在 `observation_keep_time 0` 時會一直重標最後一包 cloud（所以負對照要送空 cloud 而不是停止發送）。(2) 跑兩個 ROS 整合檔在同一個 pytest 行程會 `rclpy.init` 兩次而失敗，並漏出 `boid_controller` 到同一個 ROS domain，污染之後的 sanity（boid 提前移動、capture 在 target 發佈前）；清理程序要用 PID／含 worktree 路徑的精確 pattern，跑 benchmark 前先確認 domain 乾淨。

---

## Phase 8 — 控制面板、smart 逃跑者、優勢區域選擇器（2026-10-08）

**交付：**視窗內控制面板改版：三個模式按鈕（Baseline / Sensor + ROS / Nav2 target）、sim 內的 `StackSupervisor` 在背景以 `swarm_stack.launch.py` 重啟 swarm stack（視窗不關）、五個分頁（欄位分「即時」與「需重啟」，Apply & restart）、`ui_scale`（預設 1.5）、可捲動與縮放；話題 `/ui/mode_request`、`/ui/stack_status`。新逃跑者 `evader:=smart`（`behaviors/smart_evader.py`）：視窗預設，headless 預設仍 reactive。優勢區域選擇器（`behaviors/dominance.py`）由 smart 與 Nav2 共用，Nav2 加路徑交接平滑化（`smooth_start`）。UI 模式安全出生（`spawn.py`：`spawn_safe`、`spawn_min_clearance`、`capture_grace`）；另修 SIGINT（`ensure_sigint_deliverable`）與 agent 繪製放大。

**診斷：**(1) 撞牆不是退化，是 ReactiveEvader 的設計（無脫牆邏輯）。(2) Sensor+ROS 逃跑者「笨」不是模式問題：兩模式行為相同；真因是 `_free_pos` 抽 200 次失敗就退回地圖中心，12 隻時約一半 seed 一開局就被包圍。(3) Nav2 原地轉向的根因是 RPP 的 rotate-to-heading，不是 goal preempt。

**量測（非預先登記，n 小）：**Nav2 純 nav2 mode 速度 0.31→2.53 m/s、plan 失敗 24/49→3/43（seeds 1-5）；純 nav2 時間比例仍只有約 11%。obstacle_field、12 追兵下所有逃跑者 3–7 s 內被捕，新舊選擇器無差異；open 場地 dominance 傾向存活較久。見 `artifacts/evader-compare-2026-10-08/`、`artifacts/evader-dominance-2026-10-08/`。

**限制：**UI 模式與預先登記實驗的出生規則不同，結果不可互比；Nav2 貼牆情形未在 dominance 對照中重測。測試收集數：單元 454、ROS 整合 23。

