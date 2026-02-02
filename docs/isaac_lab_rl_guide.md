# Isaac Lab 強化学習ガイド

このドキュメントでは、Isaac Labを使用した強化学習の仕組みと、カスタムロボットを学習させるために必要な要素について説明します。

## 目次

1. [概要](#1-概要)
2. [必要なファイル構成](#2-必要なファイル構成)
3. [ArticulationCfg（ロボット設定）](#3-articulationcfgロボット設定)
4. [タスク環境の構成](#4-タスク環境の構成)
5. [報酬関数の設計](#5-報酬関数の設計)
6. [観測空間と行動空間](#6-観測空間と行動空間)
7. [地形・シーンの設定](#7-地形シーンの設定)
8. [実装例：4脚ロボットの段差乗り越え](#8-実装例4脚ロボットの段差乗り越え)
9. [ROS 2との統合](#9-ros-2との統合)
10. [参考リソース](#10-参考リソース)

---

## 1. 概要

### Isaac Labとは

Isaac Labは、NVIDIAが開発したロボット強化学習のためのフレームワークです。Isaac Sim上で動作し、数千の並列環境でロボットを同時にシミュレーションすることで、高速な学習を実現します。

### 強化学習の基本構成

```
環境 (Environment)
├── ロボット (ArticulationCfg)     → USD + 設定ファイル
├── シーン (SceneCfg)              → 地形、オブジェクト
├── 観測 (ObservationsCfg)         → センサ情報、状態
├── 行動 (ActionsCfg)              → モータ制御
├── 報酬 (RewardsCfg)              → 学習目標
├── 終了条件 (TerminationsCfg)     → エピソード終了
└── イベント (EventsCfg)           → リセット、カリキュラム
```

### 2つのワークフロー

| ワークフロー | 特徴 | 用途 |
|------------|------|------|
| **Manager-Based** | モジュラー設計、各機能がマネージャに分離 | プロトタイピング、チーム開発 |
| **Direct** | 単一クラスで全実装、JITコンパイル対応 | パフォーマンス重視、最適化 |

本ドキュメントでは、よりシンプルな **Manager-Based** ワークフローを中心に説明します。

---

## 2. 必要なファイル構成

カスタムロボットで強化学習を行うには、以下のファイルが必要です：

```
my_robot_rl/
├── config/
│   └── my_robot_cfg.py          # ArticulationCfg（ロボット設定）
├── tasks/
│   ├── __init__.py
│   ├── my_robot_env.py          # 環境クラス
│   └── my_robot_env_cfg.py      # 環境設定
├── assets/
│   └── my_robot.usd             # ロボットUSDファイル
└── train.py                     # 学習スクリプト
```

### 本リポジトリでの自動生成

`prepare_robot_for_isaaclab`ノードを実行すると、以下が自動生成されます：

| 生成物 | 説明 |
|--------|------|
| `diffbot.usd` | URDFから変換されたUSDファイル |
| `diffbot_cfg.py` | ros2_controlタグから生成されたArticulationCfg |

残りの環境設定ファイルは、タスクに応じて作成する必要があります。

---

## 3. ArticulationCfg（ロボット設定）

ArticulationCfgは、ロボットの物理特性とアクチュエータを定義します。

### 基本構造

```python
from isaaclab.assets import ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils

MY_ROBOT_CFG = ArticulationCfg(
    # 1. スポーン設定（USDファイルと物理特性）
    spawn=sim_utils.UsdFileCfg(
        usd_path="path/to/robot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
    ),

    # 2. 初期状態
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),  # 初期位置 [x, y, z]
        joint_pos={
            "joint_name": 0.0,  # 各ジョイントの初期角度
        },
    ),

    # 3. アクチュエータ設定
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=".*_leg_joint",  # 正規表現でジョイント選択
            stiffness=100.0,   # モータ剛性 [N·m/rad]
            damping=10.0,      # モータ減衰 [N·m·s/rad]
        ),
    },
)
```

### アクチュエータタイプ

| タイプ | 説明 | 用途 |
|--------|------|------|
| `ImplicitActuatorCfg` | PD制御ベースの暗黙的アクチュエータ | 一般的なジョイント制御 |
| `DCMotorCfg` | DCモータモデル | トルク制御が必要な場合 |
| `IdealPDActuatorCfg` | 理想的なPD制御 | シンプルな位置制御 |

---

## 4. タスク環境の構成

### 環境設定クラス (ManagerBasedRLEnvCfg)

```python
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg

@configclass
class MyRobotEnvCfg(ManagerBasedRLEnvCfg):
    """ロボットタスクの環境設定"""

    # シミュレーション設定
    sim: SimulationCfg = SimulationCfg(
        dt=0.005,              # シミュレーション時間刻み [秒]
        render_interval=4,     # レンダリング間隔
    )

    # シーン設定
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,         # 並列環境数
        env_spacing=5.0,       # 環境間の間隔 [m]
    )

    # 各マネージャ設定
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventsCfg = EventsCfg()
```

### シーン設定の詳細

```python
scene = InteractiveSceneCfg(
    num_envs=4096,
    env_spacing=5.0,

    # ロボット
    robot=ArticulationCfg(...),

    # 地形
    terrain=TerrainImporterCfg(
        terrain_type="generator",
        ...
    ),

    # センサ（オプション）
    contact_forces=ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/robot/.*_foot",
    ),
)
```

---

## 5. 報酬関数の設計

報酬関数は、ロボットに「何を達成してほしいか」を定義します。

### 報酬設定の基本構造

```python
from isaaclab.managers import RewardTermCfg

@configclass
class RewardsCfg:
    """報酬関数の設定"""

    # 正の報酬：目標達成を促進
    forward_velocity = RewardTermCfg(
        func=forward_velocity_reward,
        weight=1.0,
    )

    # 負の報酬（ペナルティ）：望ましくない動作を抑制
    action_rate = RewardTermCfg(
        func=action_rate_penalty,
        weight=-0.01,
    )

    energy_consumption = RewardTermCfg(
        func=energy_penalty,
        weight=-0.001,
    )
```

### 報酬関数の実装例

```python
def forward_velocity_reward(
    env: ManagerBasedRLEnv,
    asset_cfg: SceneEntityCfg,
    command_name: str,
) -> torch.Tensor:
    """前進速度の報酬"""
    # ロボットの現在速度を取得
    robot = env.scene[asset_cfg.name]
    velocity = robot.data.root_lin_vel_b[:, 0]  # x方向速度

    # コマンド速度を取得
    command = env.command_manager.get_command(command_name)
    target_velocity = command[:, 0]

    # 速度追従誤差を計算
    error = torch.abs(velocity - target_velocity)
    reward = torch.exp(-error / 0.25)  # 指数関数で報酬を計算

    return reward
```

### 一般的な報酬項目

#### 歩行ロボットの場合

| 報酬項目 | 重み例 | 説明 |
|----------|--------|------|
| 前進速度追従 | +1.0 | 指定速度で前進 |
| 姿勢維持 | +0.5 | 体の水平維持 |
| 足の接地 | +0.1 | 適切な歩容パターン |
| エネルギー消費 | -0.001 | 効率的な動作 |
| 動作の滑らかさ | -0.01 | 急激な動きを抑制 |
| 転倒ペナルティ | -1.0 | 転倒時の大きなペナルティ |

#### 移動ロボットの場合

| 報酬項目 | 重み例 | 説明 |
|----------|--------|------|
| 目標位置への接近 | +1.0 | ゴールに近づく |
| 経路追従 | +0.5 | 計画経路に沿う |
| 障害物回避 | +0.3 | 衝突しない |
| スムーズな旋回 | -0.01 | 急旋回を抑制 |

---

## 6. 観測空間と行動空間

### 観測空間 (Observations)

ロボットが「見える」情報を定義します。

```python
@configclass
class ObservationsCfg:
    """観測空間の設定"""

    @configclass
    class PolicyCfg(ObsGroup):
        """ポリシーネットワークへの入力"""

        # 基本状態
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)      # 基部線速度 [3]
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)      # 基部角速度 [3]
        projected_gravity = ObsTerm(func=mdp.projected_gravity)  # 重力方向 [3]

        # ジョイント状態
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)        # ジョイント位置 [N]
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)        # ジョイント速度 [N]

        # コマンド
        velocity_commands = ObsTerm(func=mdp.velocity_commands)  # 速度コマンド [3]

        # 前回のアクション
        actions = ObsTerm(func=mdp.last_action)            # 前回の行動 [N]

        def __post_init__(self):
            self.enable_corruption = True    # ノイズ付加
            self.concatenate_terms = True    # テンソル結合

    policy: PolicyCfg = PolicyCfg()
```

### 行動空間 (Actions)

ロボットを制御するコマンドを定義します。

```python
@configclass
class ActionsCfg:
    """行動空間の設定"""

    joint_pos = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*"],          # 全ジョイント
        scale=0.5,                   # スケーリング係数
        use_default_offset=True,
    )
```

#### 行動タイプ

| タイプ | 説明 | 用途 |
|--------|------|------|
| `JointPositionActionCfg` | 目標ジョイント位置 | 位置制御 |
| `JointVelocityActionCfg` | 目標ジョイント速度 | 速度制御 |
| `JointEffortActionCfg` | ジョイントトルク | トルク制御 |
| `DifferentialInverseKinematicsActionCfg` | エンドエフェクタ速度 | マニピュレーション |

---

## 7. 地形・シーンの設定

### 段差のある地形（4脚ロボット用）

```python
from isaaclab.terrains import TerrainImporterCfg, TerrainGeneratorCfg
from isaaclab.terrains.config.rough import ROUGH_TERRAINS_CFG

terrain = TerrainImporterCfg(
    prim_path="/World/ground",
    terrain_type="generator",
    terrain_generator=TerrainGeneratorCfg(
        size=(100.0, 100.0),        # 地形サイズ [m]
        border_width=10.0,
        num_rows=10,                # 行数
        num_cols=20,                # 列数
        horizontal_scale=0.1,       # 水平解像度
        vertical_scale=0.005,       # 垂直解像度
        slope_threshold=0.75,

        # カリキュラム学習
        curriculum=True,
        difficulty_range=(0.0, 1.0),

        # サブ地形の設定
        sub_terrains={
            # 平地（簡単）
            "flat": FlatTerrainCfg(
                proportion=0.2,
            ),
            # 階段（中程度）
            "stairs_up": StairsTerrainCfg(
                proportion=0.3,
                step_height_range=(0.05, 0.15),  # 段差高さ [m]
                step_width=0.3,
            ),
            "stairs_down": StairsTerrainCfg(
                proportion=0.3,
                step_height_range=(-0.15, -0.05),
                step_width=0.3,
            ),
            # ランダム凹凸（難しい）
            "random_rough": RandomRoughTerrainCfg(
                proportion=0.2,
                noise_range=(0.02, 0.10),
                noise_step=0.02,
            ),
        },
    ),
)
```

### 地形タイプ一覧

| 地形タイプ | 説明 | 難易度 |
|-----------|------|--------|
| `FlatTerrainCfg` | 平らな地面 | ★☆☆☆☆ |
| `SlopesTerrainCfg` | 傾斜面 | ★★☆☆☆ |
| `StairsTerrainCfg` | 階段 | ★★★☆☆ |
| `PyramidStairsTerrainCfg` | ピラミッド階段 | ★★★☆☆ |
| `DiscreteObstaclesTerrainCfg` | 離散障害物 | ★★★★☆ |
| `WaveTerrainCfg` | 波状地形 | ★★★★☆ |
| `RandomRoughTerrainCfg` | ランダム凹凸 | ★★★★★ |
| `HfDiscreteObstaclesTerrainCfg` | 高さマップ障害物 | ★★★★★ |

### カリキュラム学習

```python
curriculum=True,
difficulty_range=(0.0, 1.0),
```

- 学習初期は簡単な地形（difficulty=0.0）からスタート
- 性能向上に応じて徐々に難しい地形（difficulty→1.0）に移行
- 段差高さ、凹凸の激しさなどが自動調整

---

## 8. 実装例：4脚ロボットの段差乗り越え

### 完全な環境設定例

```python
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg, RewardTermCfg, TerminationTermCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg

@configclass
class QuadrupedStepClimbEnvCfg(ManagerBasedRLEnvCfg):
    """4脚ロボットの段差乗り越えタスク"""

    # === シミュレーション設定 ===
    sim = SimulationCfg(
        dt=0.005,
        render_interval=4,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
    )

    # === シーン設定 ===
    scene = InteractiveSceneCfg(
        num_envs=4096,
        env_spacing=5.0,

        # ロボット
        robot=QUADRUPED_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
        ),

        # 段差のある地形
        terrain=TerrainImporterCfg(
            terrain_type="generator",
            terrain_generator=TerrainGeneratorCfg(
                curriculum=True,
                sub_terrains={
                    "flat": FlatTerrainCfg(proportion=0.2),
                    "stairs": StairsTerrainCfg(
                        proportion=0.6,
                        step_height_range=(0.05, 0.20),
                        step_width=0.3,
                    ),
                    "rough": RandomRoughTerrainCfg(proportion=0.2),
                },
            ),
        ),

        # 足の接触センサ
        contact_forces=ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*_foot",
            history_length=3,
            track_air_time=True,
        ),
    )

    # === 観測設定 ===
    observations = ObservationsCfg()
    observations.policy = PolicyObsCfg(
        base_lin_vel=ObsTerm(func=mdp.base_lin_vel),
        base_ang_vel=ObsTerm(func=mdp.base_ang_vel),
        projected_gravity=ObsTerm(func=mdp.projected_gravity),
        velocity_commands=ObsTerm(func=mdp.generated_commands),
        joint_pos=ObsTerm(func=mdp.joint_pos_rel),
        joint_vel=ObsTerm(func=mdp.joint_vel_rel),
        actions=ObsTerm(func=mdp.last_action),
        height_scan=ObsTerm(  # 地形スキャン
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
        ),
    )

    # === 行動設定 ===
    actions = ActionsCfg()
    actions.joint_pos = JointPositionActionCfg(
        asset_name="robot",
        joint_names=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
        scale=0.25,
        use_default_offset=True,
    )

    # === 報酬設定 ===
    rewards = RewardsCfg()

    # 前進速度追従（メイン報酬）
    rewards.track_lin_vel_xy = RewardTermCfg(
        func=mdp.track_lin_vel_xy_exp,
        weight=1.0,
        params={"command_name": "base_velocity", "std": 0.5},
    )

    # 姿勢維持
    rewards.flat_orientation = RewardTermCfg(
        func=mdp.flat_orientation_l2,
        weight=0.5,
    )

    # 足の接地パターン
    rewards.feet_air_time = RewardTermCfg(
        func=mdp.feet_air_time,
        weight=0.1,
        params={"sensor_cfg": SceneEntityCfg("contact_forces")},
    )

    # エネルギー効率
    rewards.action_rate = RewardTermCfg(
        func=mdp.action_rate_l2,
        weight=-0.01,
    )

    rewards.joint_torques = RewardTermCfg(
        func=mdp.joint_torques_l2,
        weight=-0.0001,
    )

    # 転倒ペナルティ
    rewards.termination_penalty = RewardTermCfg(
        func=mdp.is_terminated,
        weight=-1.0,
    )

    # === 終了条件 ===
    terminations = TerminationsCfg()

    terminations.time_out = TerminationTermCfg(
        func=mdp.time_out,
        time_out=True,
    )

    terminations.base_contact = TerminationTermCfg(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces"), "threshold": 1.0},
    )

    # === イベント設定 ===
    events = EventsCfg()

    # リセット時のランダム化
    events.reset_base = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {},
        },
    )

    events.reset_robot_joints = EventTermCfg(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.9, 1.1),
            "velocity_range": (0.0, 0.0),
        },
    )

    # 物理パラメータのランダム化（ドメインランダマイゼーション）
    events.push_robot = EventTermCfg(
        func=mdp.push_by_setting_velocity,
        mode="interval",
        interval_range_s=(10.0, 15.0),
        params={"velocity_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5)}},
    )
```

### 学習の実行

```bash
# Isaac Labコンテナ内で実行
cd /workspace/isaaclab

# 学習開始
./isaaclab.sh -p scripts/train.py \
    --task Isaac-Velocity-Rough-Quadruped-v0 \
    --num_envs 4096 \
    --headless

# TensorBoardで進捗確認
tensorboard --logdir logs/
```

---

## 9. ROS 2との統合

### 本リポジトリでの統合アプローチ

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS 2 ワークスペース                      │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐      ┌─────────────────┐              │
│  │   URDF/Xacro    │──────│ prepare_robot   │              │
│  │  (ロボット定義)   │      │   _for_isaaclab │              │
│  └─────────────────┘      └────────┬────────┘              │
│                                    │                        │
│                          ┌─────────▼─────────┐              │
│                          │   diffbot.usd     │              │
│                          │   diffbot_cfg.py  │              │
│                          └─────────┬─────────┘              │
│                                    │                        │
│  ┌─────────────────┐      ┌────────▼────────┐              │
│  │   REST API      │◄─────│  isaaclab_api   │              │
│  │   /training/*   │      │     node        │              │
│  └────────┬────────┘      └─────────────────┘              │
│           │                                                 │
└───────────┼─────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Isaac Lab 環境                           │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐      ┌─────────────────┐              │
│  │  RL Algorithm   │◄─────│  Environment    │              │
│  │   (PPO, etc.)   │      │  (Task Config)  │              │
│  └────────┬────────┘      └─────────────────┘              │
│           │                                                 │
│           ▼                                                 │
│  ┌─────────────────┐                                        │
│  │  Trained Policy │───────────────────────────────────────►│
│  │   (checkpoint)  │         ROS 2にエクスポート可能        │
│  └─────────────────┘                                        │
└─────────────────────────────────────────────────────────────┘
```

### 学習済みポリシーのROS 2への統合

学習後、ポリシーネットワークをONNXまたはTorchScriptとしてエクスポートし、ROS 2ノードから呼び出すことができます：

```python
# 学習済みポリシーをROS 2ノードで使用
class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        # ONNXモデルをロード
        self.session = onnxruntime.InferenceSession("policy.onnx")

        # 観測データの購読
        self.obs_sub = self.create_subscription(
            Observation, '/robot/observation', self.obs_callback, 10)

        # 行動コマンドの発行
        self.cmd_pub = self.create_publisher(
            JointCommand, '/robot/command', 10)

    def obs_callback(self, msg):
        # 観測データを処理
        obs = self.process_observation(msg)

        # ポリシーネットワークで推論
        action = self.session.run(None, {"obs": obs})[0]

        # ROS 2コマンドとして発行
        self.publish_command(action)
```

---

## 10. 参考リソース

### 公式ドキュメント

- [Isaac Lab Documentation](https://isaac-sim.github.io/IsaacLab/)
- [Getting Started with Isaac Lab](https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/)
- [Isaac Lab GitHub Repository](https://github.com/isaac-sim/IsaacLab)

### 組み込みタスク例

| タスク名 | 説明 |
|----------|------|
| `Isaac-Velocity-Flat-Anymal-C-v0` | ANYmal C 平地歩行 |
| `Isaac-Velocity-Rough-Unitree-Go2-v0` | Unitree Go2 不整地歩行 |
| `Isaac-Velocity-Flat-H1-v0` | H1ヒューマノイド歩行 |
| `Isaac-Reach-Franka-v0` | Franka リーチタスク |
| `Isaac-Lift-Cube-Franka-v0` | Franka ピックアンドプレース |

### 環境一覧の確認

```bash
./isaaclab.sh -p scripts/environments/list_envs.py
```

---

## 次のステップ

1. **基本的な歩行学習**: 平地での前進速度追従から開始
2. **地形カリキュラム**: 段差、傾斜を徐々に追加
3. **センサ統合**: 高さスキャン、IMUデータの観測追加
4. **ドメインランダマイゼーション**: 物理パラメータのランダム化
5. **ROS 2統合**: 学習済みポリシーのエクスポートと実機展開
