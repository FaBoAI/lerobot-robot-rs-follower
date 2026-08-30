from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

try:
    from lerobot.cameras import CameraConfig
except ImportError:  # LeRobot version compatibility
    try:
        from lerobot.cameras.configs import CameraConfig  # type: ignore
    except ImportError:
        from typing import Any as CameraConfig  # type: ignore

try:
    from lerobot.robots import RobotConfig
except ImportError:  # LeRobot version compatibility
    from lerobot.robots.config import RobotConfig  # type: ignore


DEFAULT_MOTOR_NAMES: List[str] = [
    "gripper",
    "wrist_roll",
    "wrist_pitch",
    "wrist_yaw",
    "elbow_pitch",
    "shoulder_roll",
    "shoulder_pitch",
    "shoulder_yaw",
]

DEFAULT_MOTOR_MODELS: List[str] = [
    "RS05",
    "RS05",
    "RS05",
    "RS05",
    "RS00",
    "RS00",
    "RS06",
    "RS03",
]


@RobotConfig.register_subclass("rs_follower")
@dataclass
class RSFollowerConfig(RobotConfig):
    """
    RSFollower のコンフィグ。

    デフォルトは 7DOF + Gripper の左右 2 アーム構成です。

      LEFT side:
        0x01 : left_gripper          (RS05)
        0x02 : left_wrist_roll       (RS05)
        0x03 : left_wrist_pitch      (RS05)
        0x04 : left_wrist_yaw        (RS05)
        0x05 : left_elbow_pitch      (RS00)
        0x06 : left_shoulder_roll    (RS00)
        0x07 : left_shoulder_pitch   (RS06)
        0x08 : left_shoulder_yaw     (RS03)

      RIGHT side:
        0x11 : right_gripper         (RS05)
        0x12 : right_wrist_roll      (RS05)
        0x13 : right_wrist_pitch     (RS05)
        0x14 : right_wrist_yaw       (RS05)
        0x15 : right_elbow_pitch     (RS00)
        0x16 : right_shoulder_roll   (RS00)
        0x17 : right_shoulder_pitch  (RS06)
        0x18 : right_shoulder_yaw    (RS03)

    motor_names / motor_models / left_motor_ids / right_motor_ids を設定ファイルで
    上書きすると、ID・軸数・モデル構成を変更できます。
    """

    cameras: Dict[str, CameraConfig] = field(default_factory=dict)


    # YAML configuration.
    # config_file を指定するとその YAML を読み込みます。未指定の場合は、
    # load_default_yaml=True のときパッケージ同梱 configs/<config_name> を読み込みます。
    config_file: Optional[str] = None
    config_name: str = "rs_follower_7dof_gripper.yaml"
    load_default_yaml: bool = True

    # SocketCAN channel
    channel: str = "can0"

    # モータ構成。motor_names と motor_models は同じ長さにしてください。
    motor_names: List[str] = field(default_factory=lambda: list(DEFAULT_MOTOR_NAMES))
    motor_models: List[str] = field(default_factory=lambda: list(DEFAULT_MOTOR_MODELS))

    # 明示 ID。None または空リストの場合は *_start_id と *_motor_count から連番生成します。
    left_motor_ids: Optional[List[int]] = field(default_factory=lambda: [
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
        0x07,
        0x08,
    ])
    right_motor_ids: Optional[List[int]] = field(default_factory=lambda: [
        0x11,
        0x12,
        0x13,
        0x14,
        0x15,
        0x16,
        0x17,
        0x18,
    ])

    left_start_id: int = 0x01
    left_motor_count: int = 8
    right_start_id: int = 0x11
    right_motor_count: int = 8

    # calibrate() で min/max を open/close として保存するとき、反転したい軸を指定します。
    # 例: ["left_shoulder_pitch", "right_elbow_pitch"] または ["elbow_pitch"]
    inverted_motor_names: List[str] = field(default_factory=list)

    # 任意で初期レンジを直指定（rad）。キーは short 名または full 名が使えます。
    # 例: {"gripper": 4.7, "left_shoulder_yaw": -1.57}
    joint_open_rad: Dict[str, float] = field(default_factory=dict)
    joint_close_rad: Dict[str, float] = field(default_factory=dict)

    # RS MIT 制御ゲイン / フィードフォワード
    kp: float = 20.0
    kd: float = 1.0
    t_ff: float = 0.0
    default_v_set: float = 0.0
    tx_hz: float = 50.0

    # ------------------------------------------------------------------
    # Gripper torque / over-current guard
    # ------------------------------------------------------------------
    # Free-closing commands are not software speed-limited.  Protection is
    # applied by a motor-side torque ceiling from startup and, after contact,
    # by closed-loop monitoring of RobStride type-2 status/fault frames.
    gripper_motor_names: List[str] = field(default_factory=lambda: ["gripper"])

    # Keep normal joint gains by default.  These optional fields are retained
    # for installations that deliberately want softer gripper gains.
    gripper_soft_gains_enabled: bool = False
    gripper_kp: Optional[float] = None
    gripper_kd: Optional[float] = None
    gripper_t_ff: Optional[float] = None

    # Motor-side safety ceiling.  The value is written to RobStride
    # limit_torque (0x700B) and read back before the gripper is enabled.
    gripper_hardware_torque_limit_enabled: bool = True
    gripper_max_torque_nm: float = 3.0
    gripper_torque_limit_required: bool = True
    gripper_torque_limit_verify: bool = True

    # RS05 current guard.  The official private-protocol range is 0..11 A.
    # 6.5 A leaves headroom for a real 3.0 N.m grasp while remaining well below
    # the motor's documented maximum phase-current value.  The limit is written
    # and read back before enable.
    gripper_hardware_current_limit_a: Optional[float] = 6.5
    gripper_current_limit_required: bool = True
    gripper_current_limit_verify: bool = True
    gripper_current_monitor_hz: float = 20.0
    gripper_current_stale_s: float = 0.25
    gripper_current_log_enabled: bool = False
    gripper_current_log_interval_s: float = 1.0
    gripper_current_soft_limit_a: Optional[float] = 5.5
    gripper_current_hard_limit_a: Optional[float] = 6.2
    gripper_current_release_a: Optional[float] = 5.0
    gripper_current_trip_confirm_count: int = 2

    # Direct type-2 status/fault monitoring.
    gripper_status_guard_enabled: bool = True
    gripper_status_stale_s: float = 0.25
    # Hold close to the requested 3.0 N.m.  The hard threshold is slightly
    # above the verified motor-side limit so normal saturation does not create
    # a false trip; actual over-current/fault flags still trip immediately.
    gripper_torque_soft_limit_nm: float = 2.85
    gripper_torque_hard_limit_nm: float = 3.15
    gripper_torque_release_nm: float = 2.70
    gripper_torque_trip_confirm_count: int = 3

    # Contact-to-hold torque profile.  The gripper closes at its normal speed
    # until contact; only the holding demand is ramped.
    gripper_contact_torque_nm: float = 1.00
    gripper_contact_initial_torque_nm: float = 2.20
    gripper_torque_ramp_nm_s: float = 3.00
    gripper_torque_backoff_nm_s: float = 2.50
    gripper_require_status_for_full_torque: bool = True
    gripper_no_status_max_torque_nm: float = 1.80

    # Peak hold torque commanded after contact.  None falls back to
    # gripper_max_torque_nm.  Keep it below the motor-side ceiling so the hold
    # command does not sit exactly on the firmware limit.
    gripper_hold_max_torque_nm: Optional[float] = None

    # Stall-protection relief (堵転保護回避).  The RS05 firmware trips its own
    # locked-rotor protection when stall current persists with no motion
    # (measured 2026-08-29: 4.8-5.2 A sustained for 1.5-2.5 s -> hard fault and
    # dead feedback).  Before that timer can fire, dip the hold torque briefly
    # so the firmware stall timer resets, then re-ramp to peak.  Pressure
    # therefore pulses: peak most of the cycle, a short dip to the rest torque.
    gripper_stall_relief_enabled: bool = False
    gripper_stall_relief_current_a: float = 4.2
    gripper_stall_relief_torque_nm: float = 2.4
    gripper_stall_relief_high_s: float = 0.7
    gripper_stall_relief_rest_s: float = 0.3
    gripper_stall_relief_rest_torque_nm: float = 1.8

    # Thermal relief.  Holding at stall current heats the RS05 fast (measured
    # ~6 C/s at 5.2 A).  While the reported temperature is above the soft
    # limit, clamp the hold torque to the relief rest torque until it cools
    # below the release value.  None disables the check.
    gripper_temp_soft_limit_c: Optional[float] = 70.0
    gripper_temp_release_c: Optional[float] = 60.0

    # Immediate protection response to overcurrent/hard-torque/fault status.
    gripper_overcurrent_backoff_rad: float = 0.05
    gripper_overcurrent_cooldown_s: float = 1.5
    gripper_overcurrent_latch_until_open: bool = True

    # Position feedback used for contact/stall detection.  Direct type-2
    # feedback is preferred; get_angle.py remains as a fallback.
    gripper_overcurrent_guard_enabled: bool = True
    gripper_guard_feedback_enabled: bool = True
    gripper_guard_feedback_hz: float = 20.0
    gripper_guard_feedback_timeout_s: float = 0.08
    gripper_guard_feedback_stale_s: float = 0.5
    gripper_guard_require_feedback: bool = True

    gripper_guard_stall_error_rad: float = 0.08
    gripper_guard_stall_motion_rad: float = 0.004
    gripper_guard_stall_confirm_count: int = 2

    # Fallback position-error guard.  With corrected RS05 +/-5.5 N.m status
    # scaling, the effective error is calculated from requested N.m / kp and
    # capped here.
    gripper_guard_hold_position_error_rad: float = 0.05
    gripper_guard_backoff_rad: float = 0.01
    gripper_guard_pressure_scale: float = 3.0
    gripper_guard_max_hold_error_rad: float = 0.15
    gripper_guard_release_rad: float = 0.12
    gripper_guard_log_interval_s: float = 1.0

    # Deprecated compatibility fields from v0.0.10.
    gripper_guard_max_position_error_rad: float = 0.0
    gripper_guard_max_closing_speed_rad_s: float = 0.0
    gripper_guard_max_opening_speed_rad_s: float = 0.0

    disable_torque_on_disconnect: bool = True

    # 目標値のステップ変化制限（[rad]）。None なら制限なし。
    max_relative_target: Optional[float] = None

    # 起動直後、最初の action で初期位置へ移動するときに、現在角度からゆっくり補間します。
    # get_angle.py で現在角度を取得できない場合は、デフォルトでは安全のため起動を止めます。
    initial_position_ramp_enabled: bool = True
    initial_position_ramp_duration_s: float = 5.0
    # 現在位置と目標の最大差がこの値 [rad] 以下ならランプを省略して即書き込み
    # (すでに初期位置にいる場合に毎回 5 秒待たないため)
    initial_position_ramp_skip_within_rad: float = 0.03
    # ---- 関節ごとのモータ側トルク上限 (limit_torque 0x700B、接続時に書込+検証) ----
    # グリッパ以外の関節に適用。full 名 → N·m。未指定の関節はモータ既定値のまま。
    # 例 (0x16 断線事故のフォローアップ、RS00 既定 14.0 の 2/3):
    #   joint_torque_limits_nm: { left_shoulder_roll: 9.3, right_shoulder_roll: 9.3 }
    joint_torque_limits_nm: Dict[str, float] = field(default_factory=dict)
    # 書込/検証に失敗したら接続を中止する (安全目的の上限なので既定 True)
    joint_torque_limit_required: bool = True

    # ---- 多回転座標系ずれへの安全装置 (2026-08-27 の 0x16 全周回転・断線事故対策) ----
    # RobStride の位置座標は ±4π の多回転絶対値で、電源サイクル後は同じ物理姿勢が
    # ±2π ずれて報告され得る。較正由来の絶対目標をそのまま使うと
    # 「同じ姿勢へ一回転して到達する」軌道になり配線を巻き込む。
    # range_check: 開始角が較正レンジ ±margin の外にある関節が1つでもあれば
    # 一切動かさずに中止する。「±2π の座標系オフセット」と「物理的な巻き込み」は
    # 数値だけでは区別できないため、曖昧なまま動かさないのが唯一安全
    # (旧 wrap_normalize (最近傍回転周への正規化) はレンジ外開始角に対して
    #  可動域の外へ巻く軌道を選び得るため 2026-08-27 に撤去)
    initial_position_range_check: bool = True
    initial_position_range_margin_rad: float = 0.5
    # 電源サイクル起因の ±2π 座標系ずれを接続時に自動検出し、セッション中の
    # 全指令に同オフセットを適用する (巻き戻し動作は一切発生しない)。
    # 休止姿勢が較正レンジのマイナス側にある関節 (0 を跨ぐ可動域) で必須
    frame_offset_adaptation: bool = True
    # max_travel: 関節あたりの移動量上限の下限値 [rad]。実際の許容値は
    # max(この値, 較正レンジのスパン + 0.5)。0 以下で無効
    initial_position_max_travel_rad: float = 1.6
    # disconnect (収録/推論の終了) 時に initial_position へゆっくり戻ってから
    # トルクを切る。initial_position 未設定またはランプ無効時は何もしない。
    return_to_initial_on_disconnect: bool = True
    initial_position_ramp_max_speed_rad_s: float = 0.5
    initial_position_ramp_interval_s: Optional[float] = None
    initial_position_read_timeout_s: float = 0.5
    initial_position_read_retries: int = 2
    initial_position_require_feedback: bool = True
    get_angle_script: Optional[str] = None

    # キャリブレーションファイルの保存先
    rs_calibration_subdir: str = "calibration/robots/rs_follower"
    calibration_dir: Optional[str] = None

    # Teleop から来る値のレンジ
    teleop_min: float = 0.0
    teleop_max: float = 100.0

    # True にすると UI 側で度表示にしたい場合などに使える（ここでは未使用）
    use_degrees: bool = False

    # ------------------------------------------------------------------
    # connect() 時の初期位置移動
    # ------------------------------------------------------------------
    # 指定すると connect() がこの姿勢へのランプ移動を完了してから返る。
    # → 推論 / teleop のループはアームが初期位置で静止した状態から始まり、
    #   最初の send_action は即時実行になる (従来の「最初の指令へ5秒ランプ」は
    #   スキップされる)。
    # キーは full name (例: left_gripper)、値は teleop 単位 (0-100)。
    # 未指定の関節は現在角度を維持。空 dict = 従来動作。
    initial_position: Dict[str, float] = field(default_factory=dict)

    # ------------------------------------------------------------------
    # 旧 6DOF 設定との互換用フィールド。
    # 古い YAML に残っていてもロードできるよう残しています。
    # ------------------------------------------------------------------
    gripper_id: Optional[int] = None
    wrist_roll_id: Optional[int] = None
    wrist_flex_id: Optional[int] = None
    elbow_flex_id: Optional[int] = None
    shoulder_lift_id: Optional[int] = None
    shoulder_pan_id: Optional[int] = None
    left_gripper_id: Optional[int] = None
    left_wrist_roll_id: Optional[int] = None
    left_wrist_flex_id: Optional[int] = None
    left_elbow_flex_id: Optional[int] = None
    left_shoulder_lift_id: Optional[int] = None
    left_shoulder_pan_id: Optional[int] = None

    gripper_open_rad: Optional[float] = None
    gripper_close_rad: Optional[float] = None
    wrist_roll_open_rad: Optional[float] = None
    wrist_roll_close_rad: Optional[float] = None
    wrist_flex_open_rad: Optional[float] = None
    wrist_flex_close_rad: Optional[float] = None
    elbow_flex_open_rad: Optional[float] = None
    elbow_flex_close_rad: Optional[float] = None
    shoulder_lift_open_rad: Optional[float] = None
    shoulder_lift_close_rad: Optional[float] = None
    shoulder_pan_open_rad: Optional[float] = None
    shoulder_pan_close_rad: Optional[float] = None
