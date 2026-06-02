# Plan A:即時 IMU → ch_robot 重定向實作計畫

## 目標

把穿戴式 IMU 即時解出的人體關節相對旋轉,直接「分解」成 ch_robot 的
17 維 `qpos`,**完全跳過 Holosoma 的 CVXPY 逐幀優化**,讓即時遙操作達到
亞毫秒級延遲。

離線資料集生成仍沿用 Holosoma(CVXPY)路徑;本計畫只取代「即時推論」這條路。

---

## 已驗證的機器人 contract(來源:MJCF)

來源檔(在 `retargeting_holosoma` 分支):
`holosoma-main/src/holosoma_retargeting/holosoma_retargeting/models/ch_robot/ch_robot_10dof.xml`

- 10-DOF。`qpos` 寬度 **17** = 7(MuJoCo freejoint:`x, y, z, qw, qx, qy, qz`)+ 10 關節。
- 關節順序(`retargeting/ch_robot_contract.py` 的 `CH_ROBOT_JOINT_NAMES`):
  `left_hip1, left_hip2, left_thigh, left_knee, left_ankle, right_hip1, right_hip2, right_thigh, right_knee, right_ankle`
- 每條腿,內→外的串接與 hinge 軸(左右相同):

  | 關節 | axis | range (rad) | actuator kp/kv |
  |---|---|---|---|
  | hip1 | X `(1 0 0)` | −1.57 ~ 1.57 | 100 / 1 |
  | hip2 | Y `(0 1 0)` | −1.57 ~ 1.57 | 100 / 1 |
  | thigh | Z `(0 0 1)` | −1.57 ~ 1.57 | 100 / 1 |
  | knee | X `(1 0 0)` | −1.57 ~ 1.57 | 80 / 1 |
  | ankle | X `(1 0 0)` | −1.57 ~ 1.57 | 20 / 1 |

- `torso` 有 `<freejoint/>`,初始 `pos="0 0 0.765"` → base 高度 ≈ **0.765 m**。

### 關鍵:座標系不一致
- IMU/人體 pipeline:**X=前、Y=左、Z=上**(`wearable_imu/ik/imu_orientation.py`)。
- 機器人 body frame:**Y=前、X=側向、Z=上**(由足部碰撞盒長邊在 Y、左右腿沿 X 分開推得)。
- 因此每個人體相對旋轉都要先做固定基底變換 `C`(human→robot):
  `R_robot = C * R_human * C⁻¹`,再做歐拉/twist 分解。
- `C` 的精確正負號/手性**必須在 MuJoCo viewer 用測試姿勢實證確認**(見 Step 5)。

---

## 資料流(即時路徑)

```
UDP 四元數封包
  → sensor/udp_receiver.LatestPacketBuffer
  → sensor/filtering.QuaternionPacketFilter
  → 套用 sensor→segment mount(imu_orientation）得 segment_orientations
  → ik/lower_body_aggregation.aggregate_lower_body_skeleton
  → LowerBodySkeleton.joint_rotations: dict[Side, LegPose(hip,knee,ankle)]
  → 【本計畫】legposes_to_qpos(...) → qpos[17]
  → UDP/管道送到 simulation policy 或 MuJoCo
```

`LegPose`(`wearable_imu/model/lower_body.py`)的 hip/knee/ankle 已是
**校準後的相對旋轉**;knee/ankle 在缺感測器時為 `None`。

---

## 目前進度(已存在的產物)

- ✅ `wearable_imu/retargeting/ch_robot_live.py`(由 Codex 產出,約 130 行)
  - `legposes_to_qpos(joint_rotations, pelvis_orientation=None, *, base_height=0.765)`
  - `twist_about_axis(rot, axis)` swing-twist
  - `HUMAN_TO_ROBOT_FRAME`(`C`)基底變換常數
  - 髖 `as_euler("XYZ")`、膝/踝 twist about X、`np.clip` 限制、wxyz 重排
- ❌ 尚未完成:測試、`__init__.py`、即時整合、`C` 實證校正、contract 對齊、qvel。

### Review:現有模組的問題清單
1. **套件命名衝突(必須先解)**:`wearable_imu/` 下的 demo 會 `sys.path.insert(0, PROJECT_ROOT=wearable_imu)`。新建 `wearable_imu/retargeting/` 會**遮蔽 repo root 的 `retargeting/`**,使得從 wearable_imu 內無法 import `retargeting.ch_robot_contract`(那份在 repo root)。
   - **建議**:把模組改放 `wearable_imu/ik/ch_robot_retarget.py`,避免遮蔽;或在 wearable_imu 內保留一份本地 `CH_ROBOT_JOINT_NAMES` 常數(以註解連結到 contract)。本計畫採「搬到 `ik/`」。
2. **沒有用 contract 的關節順序**:順序只寫在 docstring,易漂移。應 import 或本地常數化並加 assert。
3. **左右鏡像未處理**:左右腿共用同一組軸,但人體左右腿是鏡像。同一個 `C` 是否對兩腿都正確,需驗證;右腿可能需要鏡像版的 `C`(或對特定軸取負)。
4. **base 四元數策略未定**:目前直接用 pelvis 世界方向。遙操作可能只想保留 roll/pitch、去掉 heading yaw(交給策略),需做成選項。
5. **無 qvel**:策略若需 16 維 qvel,要跨幀有限差分另外算(本函式為單幀,屬整合層職責)。
6. **校準對齊未驗證**:需確認 neutral 校準姿勢 → 機器人 ~zero pose。

---

## 實作步驟

### Step 1 — 解決套件位置與 contract 對齊
- 將模組移到 `wearable_imu/ik/ch_robot_retarget.py`(避免遮蔽 repo-root `retargeting/`)。
  - 若維持在 `wearable_imu/retargeting/`,則必須新增 `__init__.py`,且接受無法直接 import repo-root contract。
- 在模組頂部定義(或從 contract import)關節順序常數,並加:
  ```python
  assert qpos.shape == (17,)
  ```
- 維持輸出 layout:`[x, y, z, qw, qx, qy, qz, 10 joints]`,關節依 `CH_ROBOT_JOINT_NAMES` 順序。

### Step 2 — 髖部 3-DOF 分解(已大致完成,確認慣例)
```python
hip1, hip2, thigh = (C * legpose.hip * C.inv()).as_euler("XYZ")  # 大寫=intrinsic,對應 X→Y→Z 串接
```
- 確認 scipy 的 intrinsic `"XYZ"` 與 MJCF 串接順序一致(hip1=X 在最內、thigh=Z 在最外)。

### Step 3 — 膝/踝 1-DOF 投影(已完成,確認軸)
```python
knee  = twist_about_axis(C * legpose.knee  * C.inv(), X_AXIS) if legpose.knee  else 0.0
ankle = twist_about_axis(C * legpose.ankle * C.inv(), X_AXIS) if legpose.ankle else 0.0
```
- 兩者 hinge 軸皆為機器人 X。`None` → 0.0。

### Step 4 — Floating base(7 維)
- 位置 `[0, 0, base_height]`(IMU 無平移量測,屬已知限制)。
- 四元數:
  - 預設:`pelvis_orientation` 經 `C` 對齊後轉 wxyz。
  - 加選項 `strip_yaw: bool`(遙操作常見需求:只保留 roll/pitch)。
- 確認輸出為 MuJoCo 的 **wxyz**(scipy `as_quat()` 是 xyzw,需重排)。

### Step 5 — 實證確定基底變換 `C`(最關鍵)
1. 在 MuJoCo viewer 載入 `ch_robot_10dof.xml`。
2. 餵入幾個已知人體姿勢(各單軸:髖前屈、髖外展、髖內外旋、屈膝、踝背屈),
   逐一檢查機器人對應關節是否動到**正確關節、正確方向**。
3. 若方向相反 → 翻 `C` 對應列的正負號;若動到錯關節 → 調整 `C` 的軸對應。
4. 確認**右腿**:若右腿外展/內旋反向,為右腿建立鏡像版 `C_right`(或對特定關節取負)。
5. 固化最終 `C`(與必要的右腿鏡像),在 docstring 記錄推導與驗證結果。

### Step 6 — 限制夾取與校準
- 全部 10 角 `np.clip(±1.57)`。
- 驗證 neutral 校準姿勢 → 10 角 ≈ 0;若有殘餘偏置,於整合層校準時吸收。

### Step 7 — 即時整合
- 在 `wearable_imu/demos/` 新增 `demo_live_retarget.py`(或擴充
  `demo_partial_imu_live_viewer.py`):每幀從 `skeleton.joint_rotations` 與
  `skeleton.segment_orientations[PELVIS]` 呼叫 retargeter,得 qpos。
- 輸出經 UDP/既有管道送到 sim/policy;跑在獨立執行緒,**不阻塞** UDP 接收。
- (可選)qvel:在整合層用相鄰幀有限差分計算 16 維 qvel。
- 量測 per-frame 時間(目標 < 1 ms)。

### Step 8 — 測試 `wearable_imu/tests/test_ch_robot_retarget.py`
- identity LegPose(全 identity)→ 10 角 ≈ 0;base = `[0,0,0.765, 1,0,0,0]`;shape `(17,)`、dtype float64。
- 已知髖旋轉 → 預期 `(hip1, hip2, thigh)` 正負號。
- `knee=None` / `ankle=None` → 0.0。
- 極端旋轉 → 輸出落在 `[-1.57, 1.57]`(確認有夾取)。
- `twist_about_axis`:純繞 X 的旋轉回傳該角;繞正交軸回傳 ≈ 0。
- 往返一致性:retarget 出的角 →(可選)用 `build_lower_body_model` FK 重建比對。
- 用 repo-root `retargeting/ch_robot_contract.py` 的 `validate_retargeted_motion`
  把 `(1,17)` qpos + 必要鍵驗過(注意 Step 1 的 import 路徑)。

### Step 9 — 對照 Holosoma(黃金標準,離線驗證)
- 取一段錄製 clip,分別跑 Plan A 與 CVXPY,逐關節比較 qpos 誤差,確認在可接受範圍。
- MuJoCo viewer 並排視覺檢查。

### Step 10 — 平滑與邊界
- 角度連續性(unwrap)、輕度低通、速率限制,避免抖動/超速指令。

---

## 風險與開放問題
- **`C` 的手性/正負號**(最易錯)→ Step 5 實證解決。
- **右腿鏡像**是否需要獨立 `C_right`。
- **歐拉萬向鎖**:髖角接近 ±90° 時 `as_euler` 可能跳變;需測極限姿勢。
- **base 無平移**:走路類動作根部不移動(既有限制)。
- **base yaw** 是否保留,取決於下游策略。
- **校準殘餘偏置**導致 zero pose 偏移。
- **套件命名衝突**(Step 1)若不處理會在整合時炸 import。

---

## 完成定義(DoD)
- [ ] 模組位置不與 repo-root `retargeting/` 衝突,可被 demo import。
- [ ] `legposes_to_qpos` 通過全部單元測試。
- [ ] `C`(含右腿處理)經 MuJoCo viewer 實證確認。
- [ ] live demo 能即時輸出 qpos 並驅動 MuJoCo/policy,延遲 < 1 ms/frame。
- [ ] 與 Holosoma 離線對照誤差在可接受範圍。
```
