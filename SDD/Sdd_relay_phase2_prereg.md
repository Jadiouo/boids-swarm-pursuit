# SDD Addendum — Sighting Relay Phase 2：公平比較與預先登記實驗

- Date: 2026-10-08
- Status: **pre-registered**（本檔在正式實驗執行前定稿；實驗後不得因結果改場景、格數或判準。任何偏離須在結果報告中逐條列出。）
- 延續：`Sdd_distributed_sighting_relay_v1.md`（phase 1）
- 依據：紅方第 1 輪與診斷實驗 `artifacts/sighting-relay-diagnostic-2026-10-08/`

## 1. 為什麼要做 phase 2
Phase 1 的評估有三個結構性問題（已核實）：
1. oracle 用 `comm_range` 12m 多跳連通分量，ros 用 `radio_range` 8m 單跳，比較不公平。
2. 4 agents / 30s 的場景三種模式都是 0 捕獲（診斷 0/15），無法用任務結果比較。
3. shared sighting QoS `BEST_EFFORT depth=1` 會自造丟包：同程序 depth=1 collector 只看到 depth=300 collector 的 64–71% keys（n=4）；事件式丟包上界 41%。

另外，同一 seed 結果不決定性（例：同設定 3 次中 2 次捕獲），因此**每一次 run 都是一個獨立樣本**，seed 只用於控制出生點分佈，不宣稱可重現。

## 2. 實作需求（R-xx）
- **R-01** `shared_sighting_qos_depth`（int，預設 1 以保留 phase 1 行為）：controller 的 shared sighting 訂閱與發佈 QoS depth。
- **R-02** `oracle_max_hops`（int，0 = 不限（現行多跳），1 = 單跳）：`comms.propagate_sightings` 支援跳數上限；單元測試涵蓋鏈狀拓樸（A–B–C，A 看到，max_hops=1 時 C 不知道）。
- **R-03** 計數器：每個 controller 在 status 輸出 `sightings_published`、`sightings_received_total`、`sightings_received_in_range`、`sightings_received_out_of_range`、`sightings_received_self`；模擬器輸出每個 sighting 發佈時刻的 agent 距離矩陣或「應收者集合」（在 radio_range 內的 agent），使 per-receiver 丟包率 = 1 − in_range_received / in_range_expected 可算。
- **R-04** 實驗執行器支援下列實驗矩陣、每格 N 次獨立 run、序列執行（不平行；平行在診斷中未證實安全）、記錄 source fingerprint 與 git commit、每格 Wilson 95% 區間。
- **R-05** 分析與圖：捕獲率長條圖（附 Wilson 區間）、捕獲時間分佈、track-valid fraction、per-receiver 丟包率；輸出 PNG 至 `docs/media/` 與 summary 至 artifacts。

## 3. 預先登記的實驗 E1
**共同設定**：strategy `intercept`、env `open`、perception `sensor`、capture `hull`、time_limit 30s、episodes 1、`comm_range = radio_range = 8.0`、其餘為 `params.yaml` 出貨值。

**場景**
- **S12**：num_agents 12 —— 主要場景（診斷顯示可捕獲但非必然）。
- **S4**：num_agents 4 —— 困難場景，預期接近 0 捕獲；照樣報告。

**模式（5 格）**
| 代號 | sharing | 說明 |
|---|---|---|
| off | 不分享 | 只有自身感測 |
| oracle-mh | oracle，`oracle_max_hops=0` | 模擬器真值、多跳 |
| oracle-1h | oracle，`oracle_max_hops=1` | 模擬器真值、單跳（拆開「多跳」效應） |
| ros-d1 | ros，depth 1 | phase 1 行為 |
| ros-d10 | ros，depth 10 | 拆開「QoS 自造丟包」效應 |

**樣本數**：S12 每格 20 runs（seeds 1–20）；S4 每格 10 runs（seeds 1–10）。總 150 runs，序列執行。

**主要指標**：捕獲率（Wilson 95%）。**次要**：捕獲時間中位數、track-valid fraction、per-receiver 丟包率（ros 模式）。

**預先陳述的假說與判準**
- H1：ros-d10 的 per-receiver 丟包率低於 ros-d1（判準：兩者丟包率差 > 10 個百分點）。
- H2：S12 捕獲率 off ≤ ros ≤ oracle-1h ≤ oracle-mh 的排序。**僅描述性**：n=20 時 Wilson 區間寬，區間重疊即報「無法區分」，不宣稱優劣。
- H3：ros-d10 與 oracle-1h 的捕獲率差距 < ros-d1 與 oracle-1h 的差距（描述性）。

**不做**：不調整場景直到某模式勝出；不刪除逾時或當機的 run（當機 run 記為 invalid 並報告數量；invalid > 10% 則該格結果標為不可靠）。

## 4. 後續（不在本實驗範圍）
- E2：{reactive, nav2} evader × {off, oracle-1h, ros-d10}，待 M7 Nav2 spike 結果決定。
- 延遲／丟包模型、多跳 relay、共變異數加權融合：延後。
