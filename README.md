# lerobot_robot_rs_follower

LeRobot 用 RobStride/QDD follower プラグインです。

**LeRobot 0.6.x 対応済み** (2026-08-12 に 0.6.0 実環境で結合検証済み:
プラグイン自動発見 / config 解決 / インスタンス生成 / `lerobot_camera_hsb`
カメラプラグインとの統合)。

## インストール (LeRobot 0.6.x)

```bash
cd rs-follower
pip install -e . --no-deps
pip install python-can PyYAML   # 未導入の場合
```

- **`--no-deps` 必須 (Jetson)**: pip の依存解決が numpy 等をダウングレードし
  CUDA ビルドの torch スタックを壊すことがあるため。
- LeRobot 0.6.x は Python >= 3.12 が必要。
- インストール後は `--robot.type=rs_follower` がそのまま使える(自動発見)。

```bash
lerobot-teleoperate \
    --robot.type=rs_follower \
    --robot.id=my_rs \
    --teleop.type=otter_leader \
    --teleop.port=/dev/ttyUSB0 ...

# HSB カメラ (VB1940) と組み合わせる場合
lerobot-record \
    --robot.type=rs_follower \
    --robot.cameras='{"front": {"type": "hsb", "camera_mode": 1}}' \
    --teleop.type=otter_leader ...
```

## 7DOF + Gripper default map

| Side | ID | Model | Name |
|---|---:|---|---|
| left | 0x01 | RS05 | left_gripper |
| left | 0x02 | RS05 | left_wrist_roll |
| left | 0x03 | RS05 | left_wrist_pitch |
| left | 0x04 | RS05 | left_wrist_yaw |
| left | 0x05 | RS00 | left_elbow_pitch |
| left | 0x06 | RS00 | left_shoulder_roll |
| left | 0x07 | RS06 | left_shoulder_pitch |
| left | 0x08 | RS03 | left_shoulder_yaw |
| right | 0x11 | RS05 | right_gripper |
| right | 0x12 | RS05 | right_wrist_roll |
| right | 0x13 | RS05 | right_wrist_pitch |
| right | 0x14 | RS05 | right_wrist_yaw |
| right | 0x15 | RS00 | right_elbow_pitch |
| right | 0x16 | RS00 | right_shoulder_roll |
| right | 0x17 | RS06 | right_shoulder_pitch |
| right | 0x18 | RS03 | right_shoulder_yaw |

`right_shoulder_yaw` は `0x18` です。`0x08` は `left_shoulder_yaw` と重複します。

## Config fields

```yaml
motor_names:
  - gripper
  - wrist_roll
  - wrist_pitch
  - wrist_yaw
  - elbow_pitch
  - shoulder_roll
  - shoulder_pitch
  - shoulder_yaw

motor_models:
  - RS05
  - RS05
  - RS05
  - RS05
  - RS00
  - RS00
  - RS06
  - RS03

left_motor_ids:  [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
right_motor_ids: [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18]
```

軸数や ID を変えたい場合は、`motor_names` / `motor_models` / `left_motor_ids` / `right_motor_ids` の長さを合わせて変更してください。

明示 ID を使わず連番生成にする場合は、`left_motor_ids: []` / `right_motor_ids: []` とし、`left_start_id` / `left_motor_count` / `right_start_id` / `right_motor_count` を設定します。

```yaml
left_motor_ids: []
right_motor_ids: []
left_start_id: 0x01
left_motor_count: 8
right_start_id: 0x11
right_motor_count: 8
```


## YAML configuration

This package now installs the default YAML at:

```text
lerobot_robot_rs_follower/configs/rs_follower_7dof_gripper.yaml
```

At startup, RSFollower loads configuration in this priority order:

1. `config_file` field, if specified
2. `RS_FOLLOWER_CONFIG` environment variable, if specified
3. packaged `configs/rs_follower_7dof_gripper.yaml` when `load_default_yaml: true`

The packaged default YAML includes the current mirrored-axis settings:

```yaml
inverted_motor_names:
  - right_shoulder_yaw
  - right_shoulder_pitch
  - right_shoulder_roll
  - right_wrist_pitch
  - left_gripper
```

After changing `inverted_motor_names`, remove the old calibration JSON and recalibrate.

## Safe initial-position ramp

起動直後、最初の `send_action()` で follower が初期位置へ一気に移動しないように、現在角度から目標初期位置までソフトウェア補間する処理を追加しています。

デフォルトでは `~/RS/get_angle.py` で各モータの現在角度を読み、最初の action だけ以下の設定に従ってゆっくり移動します。

```yaml
initial_position_ramp_enabled: true
initial_position_ramp_duration_s: 5.0
initial_position_ramp_max_speed_rad_s: 0.5
initial_position_ramp_interval_s: null
initial_position_read_timeout_s: 0.5
initial_position_read_retries: 2
initial_position_require_feedback: true
# get_angle_script: ~/RS/get_angle.py
```

- `initial_position_ramp_duration_s`: 最低でもこの秒数をかけて初期位置へ移動します。
- `initial_position_ramp_max_speed_rad_s`: 最大移動速度です。移動量が大きい場合は、この速度以下になるよう移動時間を自動で延ばします。
- `initial_position_ramp_interval_s`: 補間送信周期です。`null` の場合は `tx_hz` から計算します。
- `initial_position_require_feedback`: `true` の場合、現在角度を取得できない軸があると安全のため起動を停止します。どうしても従来動作に近いフォールバックを使う場合だけ `false` にしてください。

初回 `send_action()` は、初期位置へのスロー移動が完了するまでブロックします。


## Gripper 3.0 N·m / RS05 scale fix / current guard (v0.0.17)

v0.0.17 は、500 mL PET ボトルなどを保持するときに v0.0.16 の握力が早く抑えられる問題を修正しています。

v0.0.16 以前の `robstride_bus.py` では RS05 の Type-2 トルクフィードバックを誤って `-17..+17 N·m` で換算していました。RS05 の正しい private-protocol スケールは `-5.5..+5.5 N·m` です。この誤差により、旧版の `2.70 N·m` soft threshold は、実トルクがおよそ `0.87 N·m` の段階で発動していました。

v0.0.17 では次のように修正しています。

1. RS05 の command/status トルク換算を `±5.5 N·m`、速度換算を `±50 rad/s` に修正します。
2. 左右グリッパの `limit_torque` (`0x700B`) を `3.0 N·m` に設定し、読み戻し後に enable します。
3. `limit_cur` (`0x7018`) を `6.5 A` に設定し、読み戻し後に enable します。
4. `iqf` (`0x701A`) を `20 Hz` で監視し、`5.5 A` から握り増しを抑え、`6.2 A` が新しい2サンプルで続くと backoff します。
5. Type-2 実測トルクは `2.85 N·m` から握り増しを抑えます。`3.15 N·m` の連続検出、モーターの overcurrent、gridlock/stall-current、温度・ドライバ fault は hard trip です。
6. 接触後は `2.20 N·m` 相当から開始し、正常な新しいトルク・電流フィードバックがある間だけ最大 `3.0 N·m` へ上げます。
7. hard trip 時は `0.05 rad` 開く方向へ戻し、クールダウンと明示的な開く操作が終わるまで再閉鎖をラッチします。
8. 物体に触れる前の Leader 指令と自由閉じ速度は変更しません。

主な設定です。

```yaml
gripper_hardware_torque_limit_enabled: true
gripper_max_torque_nm: 3.0
gripper_torque_limit_required: true
gripper_torque_limit_verify: true

gripper_hardware_current_limit_a: 6.5
gripper_current_limit_required: true
gripper_current_limit_verify: true
gripper_current_monitor_hz: 20.0
gripper_current_stale_s: 0.25
gripper_current_soft_limit_a: 5.5
gripper_current_hard_limit_a: 6.2
gripper_current_release_a: 5.0
gripper_current_trip_confirm_count: 2

gripper_status_guard_enabled: true
gripper_status_stale_s: 0.25
gripper_torque_soft_limit_nm: 2.85
gripper_torque_hard_limit_nm: 3.15
gripper_torque_release_nm: 2.70
gripper_torque_trip_confirm_count: 3

gripper_contact_torque_nm: 1.00
gripper_contact_initial_torque_nm: 2.20
gripper_torque_ramp_nm_s: 3.00
gripper_torque_backoff_nm_s: 2.50

gripper_require_status_for_full_torque: true
gripper_no_status_max_torque_nm: 1.80

gripper_overcurrent_backoff_rad: 0.05
gripper_overcurrent_cooldown_s: 1.5
gripper_overcurrent_latch_until_open: true
```

### 電流・トルクのログ

実機調整時は一時的に次を有効にすると、`iqf`、Type-2 トルク、温度、status age を確認できます。

```yaml
gripper_current_log_enabled: true
gripper_current_log_interval_s: 0.5
```

確認後はログ量を抑えるため `false` に戻せます。

### 重要な安全上の注意

RS05 の定格連続トルクはピークトルクより低いため、`3.0 N·m` は長時間の連続把持ではなく、必要な時間だけ使ってください。モーター温度の上昇、異音、過電流ラッチが見られた場合は、直ちに物体を外し、次のように下げます。

```yaml
gripper_max_torque_nm: 2.5
gripper_torque_soft_limit_nm: 2.35
gripper_torque_hard_limit_nm: 2.65
gripper_contact_initial_torque_nm: 1.8
gripper_hardware_current_limit_a: 5.5
gripper_current_soft_limit_a: 4.7
gripper_current_hard_limit_a: 5.3
```

円筒形で表面が滑る PET ボトルでは、トルクだけでなく指先摩擦が支配的になる場合があります。シリコン/ゴムパッド、ボトル径に沿う凹形状、指先間の平行度を改善すると、モーター負荷を増やさず保持力を上げられます。

## v0.0.17: corrected RS05 torque units and verified current ceiling

- Corrected RS05 operation-command and Type-2 status scaling from the erroneous `±17 N.m` to `±5.5 N.m`.
- Corrected RS05 velocity scaling from `±33 rad/s` to `±50 rad/s`.
- The previous `2.70 N.m` soft threshold had effectively activated near `0.87 N.m`; it now operates in real RS05 torque units.
- Kept the requested motor-side maximum at `3.0 N.m` rather than raising it again.
- Added required, verified startup programming of `limit_cur = 6.5 A`.
- Added `20 Hz` `iqf` monitoring with `5.5 A` soft, `6.2 A` hard and `5.0 A` release thresholds.
- Increased contact hold start to `2.20 N.m` and ramp to `3.00 N.m/s`, while preserving free-closing speed.
- Fixed Type-21 gridlock/stall-current decoding to use fault-value bit 14.
- A zero-valued Type-21 report is no longer treated as a hard fault merely because a report was received.
- Added RS05 configuration validation: torque cannot exceed `5.5 N.m`, current cannot exceed `11 A`.

## v0.0.16: 3.0 N·m with motor/current monitoring

- RS05 の `limit_torque` を `3.0 N·m` に設定し、enable 前に読み戻して検証します。
- Type-2 status のトルク、過電流、stall-current、温度・エンコーダ等の fault を監視します。
- `iqf` 電流値を `10 Hz` で取得し、設定した場合は soft/hard 電流しきい値にも利用します。
- 通常保持のソフト制限を `2.70 N·m`、ソフトウェア hard trip を `2.95 N·m`、モーター側上限を `3.0 N·m` に分けました。
- hard trip 時は `0.04 rad` 開く指令と閉じラッチを入れます。
- 同じ CAN 状態フレームを二重カウントせず、新しい2フレームで判定します。
- フィードバック消失中はラッチを勝手に解除せず、フル保持力も許可しません。
- Type-2 の位置情報が新しい間はそれを使用し、`get_angle.py` はフォールバック時だけ実行します。
- グリッパ以外の既存関節の command scaling と自由閉じ速度は変更していません。

## v0.0.15: stronger gripper hold

- 自由に閉じている間の速度と通常ゲインは変更していません。
- 接触検出後の保持スケールを `3.0` に引き上げました。
- 実効保持位置誤差のハード上限を `0.10 rad` から `0.12 rad` に変更しました。
- v0.0.14 より接触後の保持量を 20% 増やしています。
- 過電流、モータエラー、急な温度上昇が出る場合は、`pressure_scale: 2.5` / `max_hold_error_rad: 0.10` に戻してください。

## v0.0.14: stronger gripper hold

- 自由に閉じている間の速度と通常ゲインは変更していません。
- 接触検出後の保持スケールを `2.5` に引き上げました。
- 実効保持位置誤差には `0.10 rad` のハード上限を追加しました。

## Install

LeRobot の古い版では、`pip install -e .` で入る PEP 660 editable install が
サードパーティロボット探索に見えないことがあります。
`--robot.type` の候補に `rs_follower` が出ない場合は、editable ではなく通常 install を使ってください。

```bash
conda activate lerobot
cd ~/RS/rs-follower
pip uninstall -y lerobot_robot_rs_follower
pip install --no-cache-dir .

# 確認
python - <<'PY2'
import lerobot_robot_rs_follower
from lerobot_robot_rs_follower import RSFollowerConfig
print("plugin import OK: rs_follower")
print(RSFollowerConfig)
PY2

lerobot-teleoperate --help | grep -E "rs_follower|robot.type"
```

開発用に editable install したい場合は、古い LeRobot では以下の互換モードを使ってください。

```bash
pip uninstall -y lerobot_robot_rs_follower
pip install -e . --config-settings editable_mode=compat
```

## LeRobot に `rs_follower` が表示されない場合

LeRobot は、インストール済み Python パッケージのうち、パッケージ名が
`lerobot_robot_` で始まるものを自動探索します。`--robot.type` の候補に
`rs_follower` が出ない場合は、プラグインの import 時に例外が出て
登録処理まで進んでいない可能性があります。

この版では以下を修正しています。

- `__init__.py` で `RSFollowerConfig` を先に import し、
  `RobotConfig.register_subclass("rs_follower")` が探索時に必ず実行されるように変更
- `RSFollower` 本体は lazy import に変更し、探索時に CAN 依存の import 失敗で
  `rs_follower` が非表示にならないように変更
- `install_requires` に `lerobot` と `python-can` を追加

確認コマンド:

```bash
python - <<'PY'
import lerobot_robot_rs_follower
from lerobot.robots import RobotConfig
print('plugin import OK')
print(RobotConfig.get_choice_name(lerobot_robot_rs_follower.RSFollowerConfig) if hasattr(RobotConfig, 'get_choice_name') else 'rs_follower registered')
PY

lerobot-teleoperate --help | grep -A1 'robot.type'
```

`--robot.type` の候補に `rs_follower` が出れば認識されています。
