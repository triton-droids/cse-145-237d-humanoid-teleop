# Triton 人形機器人步態模擬系統

> 基於強化學習的人形機器人步態控制系統，建構於 NVIDIA Isaac Lab 之上，由加州大學聖地牙哥分校 Triton Droids 機器人社開發。

---

## 目錄

- [Triton 人形機器人步態模擬系統](#triton-人形機器人步態模擬系統)
  - [目錄](#目錄)
  - [專案簡介](#專案簡介)
  - [程式庫結構](#程式庫結構)
  - [機器人](#機器人)
  - [強化學習環境架構](#強化學習環境架構)
  - [觀測與動作空間](#觀測與動作空間)
    - [觀測（42–44 維）](#觀測4244-維)
    - [動作（10 維）](#動作10-維)
  - [獎勵系統](#獎勵系統)
    - [速度追蹤（越高越好）](#速度追蹤越高越好)
    - [穩定性（懲罰項——越接近零越好）](#穩定性懲罰項越接近零越好)
    - [步態品質](#步態品質)
    - [平滑性 / 效率](#平滑性--效率)
    - [存活獎勵](#存活獎勵)
  - [課程學習](#課程學習)
  - [自適應領域隨機化（ADR）](#自適應領域隨機化adr)
  - [馬達延遲建模](#馬達延遲建模)
  - [支援的強化學習演算法](#支援的強化學習演算法)
  - [訓練流程](#訓練流程)
  - [安裝與執行](#安裝與執行)
    - [環境需求](#環境需求)
    - [安裝](#安裝)
    - [驗證安裝](#驗證安裝)
    - [訓練](#訓練)
    - [評估檢查點](#評估檢查點)
    - [其他演算法](#其他演算法)
  - [關鍵設計決策](#關鍵設計決策)

---

## 專案簡介

本專案在大規模並行物理模擬環境中，利用深度強化學習訓練雙足人形機器人**行走**。最終目標是模擬到現實的遷移（sim-to-real transfer）：在模擬中學到的策略可以直接部署到真實硬體上。

**核心技術棧：**

- **模擬器**：NVIDIA Isaac Lab（Omniverse / PhysX 後端）
- **並行規模**：GPU 上同時運行 4,096 個物理環境
- **演算法**：PPO（近端策略最佳化），支援 4 種不同的強化學習函式庫
- **物理更新率**：200 Hz｜**控制更新率**：50 Hz

**已註冊任務：**
| Gym ID | 用途 |
|---|---|
| `Isaac-Humanoid-Locomotion-Flat-Direct-v0` | 行走（前進、側移、轉向） |
| `Isaac-Humanoid-Standing-Flat-Direct-v0` | 原地平衡站立 |

---

## 程式庫結構

```
simulation/
├── source/tritonhumanoid/           # 主要 Python 套件（pip install -e）
│   └── tritonhumanoid/
│       ├── tasks/direct/tritonhumanoid/
│       │   ├── tritonhumanoid_env.py        # 核心 RL 環境（約 1,700 行）
│       │   ├── tritonhumanoid_env_cfg.py    # 超參數、獎勵、ADR 設定
│       │   ├── standing_env.py              # 站立／平衡變體
│       │   ├── standing_env_cfg.py
│       │   └── agents/                      # 各演算法 PPO 設定檔
│       │       ├── rl_games_ppo_cfg.yaml
│       │       ├── rsl_rl_ppo_cfg.py
│       │       ├── skrl_ppo_cfg.yaml
│       │       ├── skrl_amp_cfg.yaml
│       │       └── sb3_ppo_cfg.yaml
│       ├── assets/
│       │   ├── humanoid.py                  # 機器人資產與致動器定義
│       │   ├── human_offset_corrected.urdf  # 使用中的機器人模型（運動學修正版）
│       │   └── robot_meshes/                # 視覺網格
│       └── utils/
│           └── convert_urdf_usd.py          # URDF → USD 轉換工具
│
├── scripts/                         # 訓練與評估入口
│   ├── rl_games/   train.py  play.py
│   ├── rsl_rl/     train.py  play.py
│   ├── skrl/       train.py  play.py
│   ├── sb3/        train.py  play.py
│   ├── zero_agent.py                # 基礎測試：全零動作
│   └── random_agent.py              # 基礎測試：隨機動作
│
├── logs/                            # TensorBoard 日誌與各次執行的檢查點
├── outputs/                         # Hydra 設定傾印（每次執行一份）
├── run_train_locomotion.sh          # 快速啟動訓練腳本
└── README.md
```

---

## 機器人

自製 10 自由度（DOF）雙足人形機器人，每條腿有 5 個驅動關節：

```
軀幹（浮動基座）
├── 左腿
│   ├── hip1   （冠狀面外展／內收）
│   ├── hip2   （矢狀面屈曲／伸展）
│   ├── thigh  （矢狀面）
│   ├── knee   （屈曲／伸展）
│   └── ankle  （背屈／蹠屈）
└── 右腿（左腿的鏡像）
```

目前使用的 URDF 為 `human_offset_corrected.urdf`——經過運動學修正、關節座標系正確對齊的版本，確保模擬穩定性。

---

## 強化學習環境架構

環境繼承自 Isaac Lab 的 `DirectRLEnv`，並覆寫標準回呼函式：

```
_setup_scene()        → 生成 4096 個機器人與地形
_pre_physics_step()   → 套用 PD 關節指令
_apply_action()       → 將目標值寫入致動器
_get_observations()   → 建構觀測向量（詳見下方）
_get_rewards()        → 計算 30+ 個獎勵項
_get_dones()          → 偵測跌倒 / 逾時
_reset_idx()          → 隨機化初始狀態
```

**物理時間步**：5 ms（200 Hz）
**控制抽取倍率**：4x → 策略有效更新率 = 50 Hz
**回合長度**：1,000 步（50 Hz 下約 20 秒）

---

## 觀測與動作空間

### 觀測（42–44 維）

| 群組                 | 內容                   | 維度 |
| -------------------- | ---------------------- | ---- |
| 基座線速度           | 縮放係數 2.0           | 3    |
| 基座角速度           | 縮放係數 0.25          | 3    |
| 投影重力向量         | 機體座標系中的單位向量 | 3    |
| 速度指令             | (vx, vy, 偏航角速度)   | 3    |
| 關節位置             | 相對於預設姿態的偏差   | 10   |
| 關節速度             | 縮放係數 0.05          | 10   |
| 前一步動作           | 上一次策略輸出         | 10   |
| 步態相位時鐘（選用） | cos/sin 步態相位       | 2    |

### 動作（10 維）

策略輸出每個關節的**位置目標值**，傳入各關節的 PD 控制器，剛度與阻尼值由實驗調校：

| 關節群組 | 剛度 | 阻尼 |
| -------- | ---- | ---- |
| hip1     | ~80  | ~4   |
| hip2     | ~80  | ~4   |
| thigh    | ~80  | ~4   |
| knee     | ~80  | ~4   |
| ankle    | ~80  | ~4   |

---

## 獎勵系統

獎勵函數包含 30+ 個項目，分為以下類別：

### 速度追蹤（越高越好）

| 項目               | 縮放 | 說明                                 |
| ------------------ | ---- | ------------------------------------ |
| `track_lin_vel_xy` | 3.0  | 追蹤指令前進／側移速度（指數核函數） |
| `track_ang_vel_z`  | 0.6  | 追蹤指令偏航角速度                   |

### 穩定性（懲罰項——越接近零越好）

| 項目               | 縮放  | 說明               |
| ------------------ | ----- | ------------------ |
| `lin_vel_z`        | -1.5  | 懲罰垂直方向彈跳   |
| `ang_vel_xy`       | -0.05 | 懲罰滾轉／俯仰晃動 |
| `flat_orientation` | 0.6   | 獎勵軀幹保持直立   |
| `base_height`      | 可變  | 維持目標高度       |

### 步態品質

| 項目                  | 縮放 | 說明                      |
| --------------------- | ---- | ------------------------- |
| `anti_phase_hip1`     | 0.15 | 左右 hip1 關節相位差 180° |
| `foot_air_time`       | 可變 | 每步最短離地時間          |
| `foot_slip`           | -0.1 | 懲罰腳部在地面滑動        |
| `foot_contact_forces` | 可變 | 懲罰落地衝擊力過大        |

### 平滑性 / 效率

| 項目              | 縮放   | 說明                 |
| ----------------- | ------ | -------------------- |
| `action_rate`     | -0.01  | 懲罰指令突變（抖動） |
| `energy`          | -0.002 | 懲罰高關節力矩       |
| `joint_vel`       | 可變   | 懲罰關節速度過快     |
| `joint_deviation` | 可變   | 鼓勵維持預設姿態     |

### 存活獎勵

- 每步未跌倒額外獲得 **+0.05**（鼓勵更長的回合）

---

## 課程學習

代理人隨著能力提升，依序解鎖 3 個指令階段：

```
階段 0（起始）：  vx: [0.3, 1.0] m/s   vy: 0    偏航: 0
        ↓（平均累積 2,500 環境步後）
階段 1：          vx: [0.3, 1.0] m/s   vy: 0    偏航: [-0.5, 0.5] rad/s
        ↓（平均累積 5,000 環境步後）
階段 2（完整）：  vx: [0.3, 1.0] m/s   vy: [-0.3, 0.3] m/s  偏航: [-0.5, 0.5] rad/s
```

這樣可以避免代理人在尚未學會直走之前，就因為全向行走任務過難而訓練崩潰。

---

## 自適應領域隨機化（ADR）

ADR 依據代理人的成功率即時調整模擬難度，共有 100 個離散難度等級：

**成功標準**：持續達到目標前進速度

- 成功率 > 85% → 提升難度
- 成功率 < 20% → 降低難度（至少需完成 20,000 步預熱後才允許降低）

**最高難度下的隨機化參數：**

| 參數           | 範圍              |
| -------------- | ----------------- |
| 地面摩擦力     | 0.4 – 1.5         |
| 重力縮放       | 0.8 – 1.2x        |
| 機器人質量     | 0.7 – 1.3x        |
| 質心偏移       | ±2 cm             |
| 關節剛度／阻尼 | ±20%              |
| 外力推力       | 最大 ±100 N       |
| 動作噪聲       | 高斯分佈 σ ≈ 0.01 |
| 觀測延遲       | 最多 2 步         |

這使得學到的策略對真實硬體的個體差異具有強健性。

---

## 馬達延遲建模

真實伺服馬達不會即時響應。模擬器依據馬達資料集的實測分析，對**每對馬達**建模其指令延遲：

| 馬達對            | 關節        | 延遲               |
| ----------------- | ----------- | ------------------ |
| `hip1_pair_1_6`   | hip1 左+右  | 1–2 ms             |
| `hip2_pair_2_7`   | hip2 左+右  | 0–1 ms             |
| `thigh_pair_3_8`  | thigh 左+右 | 0–1 ms             |
| `knee_pair_4_9`   | knee 左+右  | 1–2 ms（延遲最大） |
| `ankle_pair_5_10` | ankle 左+右 | 0–1 ms             |

這對模擬到現實的遷移至關重要——未考慮延遲的策略在真實硬體上行為將難以預測。

---

## 支援的強化學習演算法

本專案支援 4 個 RL 函式庫，均實作 PPO：

| 函式庫                | 設定檔                  | 備註                                   |
| --------------------- | ----------------------- | -------------------------------------- |
| **RL-Games**          | `rl_games_ppo_cfg.yaml` | 主要／推薦；24 步滾動窗口，批次 24,576 |
| **RSL-RL**            | `rsl_rl_ppo_cfg.py`     | 精簡網路（32-32 隱藏層）               |
| **SKRL**              | `skrl_ppo_cfg.yaml`     | 替代 PPO 實作                          |
| **SKRL AMP**          | `skrl_amp_cfg.yaml`     | 對抗動作先驗（動作模仿）               |
| **Stable-Baselines3** | `sb3_ppo_cfg.yaml`      | 標準 SB3 PPO                           |

**RL-Games PPO 超參數（主要）：**

- 滾動視野：24 步
- 小批次大小：24,576
- 每次更新的小輪次數：5
- 熵係數：0.005
- KL 閾值（自適應學習率）：0.01
- 梯度裁剪範數：1.0
- 網路架構：MLP 400-200 隱藏單元

---

## 訓練流程

```
1. Gym 環境註冊（透過 __init__.py）
         ↓
2. 透過 Hydra 載入設定（環境 + 代理人設定合併）
         ↓
3. Isaac Lab 場景初始化（GPU 上生成 4,096 個機器人）
         ↓
4. 訓練迴圈：
   ┌──────────────────────────────────────────┐
   │  收集 24 步滾動資料（4,096 個環境）       │
   │  → 每次更新約 10 萬筆轉換資料             │
   │  計算獎勵 + 優勢估計（GAE）               │
   │  執行 5 個 PPO 小輪次                     │
   │  記錄至 TensorBoard / W&B                 │
   │  每 50 次迭代儲存一次檢查點               │
   └──────────────────────────────────────────┘
         ↓
5. 透過 play.py 評估（載入檢查點並渲染）
```

**日誌目錄結構：**

```
logs/rl_games/humanoid_flat_direct/<執行名稱>/
├── nn/          # 模型檢查點（.pth）
├── params/      # 儲存的設定（env.yaml、agent.yaml）
├── videos/      # 選用的 MP4 錄影
└── events.*     # TensorBoard 事件檔
```

---

## 安裝與執行

### 環境需求

- 支援 CUDA 的 NVIDIA GPU
- 已安裝 [Isaac Lab](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)
- Python 3.10+，Isaac Sim 4.5.0+

### 安裝

```bash
pip install -e source/tritonhumanoid
```

### 驗證安裝

```bash
python scripts/list_envs.py
python scripts/zero_agent.py --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0
```

### 訓練

```bash
# RL-Games（推薦）
python scripts/rl_games/train.py \
  --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0 \
  --num_envs=4096 \
  --headless

# 或使用便捷腳本：
./run_train_locomotion.sh
```

### 評估檢查點

```bash
python scripts/rl_games/play.py \
  --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0 \
  --checkpoint=logs/rl_games/humanoid_flat_direct/<執行名稱>/nn/last_humanoid_ep_*.pth
```

### 其他演算法

```bash
python scripts/rsl_rl/train.py --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0
python scripts/skrl/train.py   --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0
python scripts/sb3/train.py    --task=Isaac-Humanoid-Locomotion-Flat-Direct-v0
```

---

## 關鍵設計決策

1. **直接強化學習**（非模型為基礎）：策略直接輸出關節位置目標值。架構簡單、訓練快速，非常適合步態控制。

2. **僅使用平坦地形**：目前地形為平面。崎嶇地形生成器的程式碼已存在但被注解掉——這是刻意為之，以保持初期訓練的穩定性。

3. **10 自由度純腿部控制**：軀幹與手臂為被動自由度。將控制集中在腿部顯著簡化了動作空間。

4. **實測致動器建模**：馬達延遲來自真實硬體資料集分析，而非臆測。這是本專案最重要的模擬到現實橋接機制之一。

5. **多演算法支援**：支援 4 個 RL 後端，方便比較演算法並在某個演算法發散或陷入瓶頸時快速切換。
