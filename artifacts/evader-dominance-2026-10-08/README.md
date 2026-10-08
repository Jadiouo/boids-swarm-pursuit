# 逃跑者優勢區域對照（sanity，非預先登記）

分支 `feat/dominance-evader`。工具：`ros2_ws/src/boids_swarm/tools/evader_compare.py`。
條件：12 agents、`spawn_safe:=true spawn_min_clearance:=6 capture_grace:=1.5`、
episodes_max 2 取第 2 回合、30 s、perception perfect。smart 的 time_scale 為預設，
Nav2 為 1.0。seeds 1-10 與 holdout 11-15 分開列（見 summary.json）。
`smart-sampled` / `nav2-sampled` = 舊的隨機候選選擇器；`smart` / `nav2` = 優勢區域。
n 很小，只能當方向參考，不是顯著性檢定。

## smart（存活秒數均值，被捕數/回合數，貼牆接觸比例）
| 組 | env | sampled | dominance |
|---|---|---|---|
| 1-10 | obstacle_field | 5.6 s, 10/10, 15% | 6.6 s, 10/10, 17% |
| 1-10 | open | 16.9 s, 5/9, 0% | 21.8 s, 3/9, 0% |
| 11-15 | obstacle_field | 6.5 s, 5/5, 53% | 5.9 s, 5/5, 43% |
| 11-15 | open | 17.8 s, 2/4, 0% | 25.4 s, 1/5, 0% |

開局 3 s 內被捕的回合已排除（early_captures 欄）。obstacle_field 兩者無明顯差異
（全部被捕，3-7 秒）；open 場地 dominance 傾向存活較久。

## Nav2（obstacle_field，seeds 1-5）
| | 純 nav2 mode 時間比例 | nav2 mode 內速度 | plan 失敗/請求 | 全程平均速度 |
|---|---|---|---|---|
| sampled（舊） | 2% | 0.31 m/s | 24/49 | 2.33 |
| dominance + 平滑化 | 11% | 2.53 m/s | 3/43 | 2.54 |

純 nav2 mode 只在最近追兵距離 >= 4 m 時出現，所以比例受幾何限制，不全由 Nav2 決定。
