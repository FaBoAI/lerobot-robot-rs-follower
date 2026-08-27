#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import logging
import os
import math
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from importlib import resources
from typing import Any, Dict, Iterable, List, Optional, Tuple

import can

from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot

from .config_rs_follower import DEFAULT_MOTOR_MODELS, DEFAULT_MOTOR_NAMES, RSFollowerConfig
from .robstride_bus import MotorStatus, RobStrideBus

logger = logging.getLogger(__name__)


def _load_yaml_file(path: Path) -> Dict[str, Any]:
    """Load a YAML file as a dict. PyYAML is optional until this is used."""
    try:
        import yaml  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "YAML config support requires PyYAML. Install it with: pip install PyYAML"
        ) from exc

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML config must be a mapping/object: {path}")
    # LeRobot configs are sometimes nested under `robot:`.
    robot_data = data.get("robot")
    if isinstance(robot_data, dict):
        data = robot_data
    return data


def _default_yaml_config_path(config_name: str) -> Optional[Path]:
    try:
        resource = resources.files("lerobot_robot_rs_follower").joinpath(
            "configs", config_name
        )
        if resource.is_file():
            return Path(str(resource))
    except Exception:
        logger.debug("RSFollower: packaged YAML config not found", exc_info=True)
    return None


def _apply_yaml_config_overrides(cfg: RSFollowerConfig) -> None:
    """Apply RSFollowerConfig values from YAML before motor specs are built.

    Priority:
      1. cfg.config_file
      2. RS_FOLLOWER_CONFIG environment variable
      3. packaged configs/<cfg.config_name>, when cfg.load_default_yaml is true
    """
    path_text = getattr(cfg, "config_file", None) or os.environ.get("RS_FOLLOWER_CONFIG")
    path: Optional[Path] = Path(path_text).expanduser() if path_text else None

    if path is None and getattr(cfg, "load_default_yaml", True):
        path = _default_yaml_config_path(getattr(cfg, "config_name", "rs_follower_7dof_gripper.yaml"))

    if path is None:
        return
    if not path.exists():
        raise FileNotFoundError(f"RSFollower YAML config not found: {path}")

    data = _load_yaml_file(path)
    ignored = {"type", "robot_type"}
    applied: Dict[str, Any] = {}
    for key, value in data.items():
        if key in ignored:
            continue
        if hasattr(cfg, key):
            setattr(cfg, key, value)
            applied[key] = value
        else:
            logger.warning("RSFollower YAML: unknown key ignored: %s", key)

    logger.info("RSFollower: loaded YAML config from %s; applied keys=%s", path, sorted(applied))


# =============================================================================
# RobStride / QDD MIT-style CAN helpers
# =============================================================================


def _float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """float in [x_min, x_max] -> unsigned int (0..(1<<bits)-1)."""
    span = x_max - x_min
    offset = x_min
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return int((x - offset) * ((float((1 << bits) - 1)) / span))


def _build_ext_id(mode: int, data16: int, dev_id: int) -> int:
    """
    RobStride private protocol 29-bit ExtID layout:
        29-bit ExtID = (mode[4:0] << 24) | (data16 << 8) | (id8)
    """
    return ((mode & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (dev_id & 0xFF)


class RS06Bus:
    """
    RS06 用 CAN バス実装（MIT / 运控モード）。

    RobStrideBus と同じように見える API を提供します:
        - connect()
        - disconnect(disable_torque: bool)
        - sync_write("Goal_Position", {"qdd0": target_rad})
    """

    P_MIN = -12.57
    P_MAX = 12.57
    V_MIN = -50.0
    V_MAX = 50.0
    KP_MIN = 0.0
    KP_MAX = 5000.0
    KD_MIN = 0.0
    KD_MAX = 500.0
    T_MIN = -36.0
    T_MAX = 36.0

    MODE_CONTROL = 1
    MODE_ENABLE = 3
    MODE_STOP = 4

    MASTER_ID = 0xFD

    def __init__(
        self,
        channel: str,
        motor_id: int,
        kp: float,
        kd: float,
        default_v_set: float,
        t_ff: float,
        tx_hz: float,
    ) -> None:
        self.channel = channel
        self.id = int(motor_id) & 0xFF
        self.kp = float(kp)
        self.kd = float(kd)
        self.default_v_set = float(default_v_set)
        self.t_ff = float(t_ff)
        self.tx_period = 1.0 / float(tx_hz) if tx_hz > 0 else 0.02

        self._bus: Optional[can.Bus] = None
        self._lock = threading.Lock()
        self._target_pos: float = 0.0
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def connect(self) -> None:
        if self._bus is not None:
            return
        self._bus = can.Bus(interface="socketcan", channel=self.channel)
        logger.info("RS06Bus(id=0x%02X): connected on %s", self.id, self.channel)
        self._running = False
        self._thread = None

    def disconnect(self, disable_torque: bool = True) -> None:
        if self._bus is None:
            return
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None

        if disable_torque:
            self._send_stop()

        self._bus.shutdown()
        self._bus = None
        logger.info("RS06Bus(id=0x%02X): disconnected", self.id)

    def sync_write(self, reg: str, values: Dict[str, float]) -> None:
        if reg != "Goal_Position":
            logger.warning("RS06Bus: unsupported reg=%s (ignored)", reg)
            return
        q = float(values.get("qdd0", 0.0))
        with self._lock:
            self._target_pos = q

        if not self._running and self._bus is not None:
            self._send_enable()
            self._running = True
            self._thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._thread.start()

    def _tx_loop(self) -> None:
        while self._running and self._bus is not None:
            with self._lock:
                p = self._target_pos
            try:
                self._send_control_frame(p)
            except Exception:
                logger.exception("RS06Bus: failed to send control frame")
            time.sleep(self.tx_period)

    def _send_enable(self) -> None:
        if self._bus is None:
            return
        extid = _build_ext_id(self.MODE_ENABLE, self.MASTER_ID, self.id)
        msg = can.Message(arbitration_id=extid, data=b"\x00" * 8, is_extended_id=True)
        try:
            self._bus.send(msg)
            logger.info("RS06Bus: send ENABLE (extid=0x%08X)", extid)
        except can.CanError:
            logger.exception("RS06Bus: ENABLE send failed")

    def _send_stop(self) -> None:
        if self._bus is None:
            return
        extid = _build_ext_id(self.MODE_STOP, self.MASTER_ID, self.id)
        msg = can.Message(arbitration_id=extid, data=b"\x00" * 8, is_extended_id=True)
        try:
            self._bus.send(msg)
            logger.info("RS06Bus: send STOP (extid=0x%08X)", extid)
        except can.CanError:
            logger.exception("RS06Bus: STOP send failed")

    def _send_control_frame(self, pos_rad: float) -> None:
        if self._bus is None:
            return

        p = float(pos_rad)
        v = float(self.default_v_set)
        kp = float(self.kp)
        kd = float(self.kd)
        tff = float(self.t_ff)

        p_uint = _float_to_uint(p, self.P_MIN, self.P_MAX, 16)
        v_uint = _float_to_uint(v, self.V_MIN, self.V_MAX, 16)
        kp_uint = _float_to_uint(kp, self.KP_MIN, self.KP_MAX, 16)
        kd_uint = _float_to_uint(kd, self.KD_MIN, self.KD_MAX, 16)
        tff_uint = _float_to_uint(tff, self.T_MIN, self.T_MAX, 16)

        extid = _build_ext_id(self.MODE_CONTROL, tff_uint, self.id)

        data = bytearray(8)
        data[0] = (p_uint >> 8) & 0xFF
        data[1] = p_uint & 0xFF
        data[2] = (v_uint >> 8) & 0xFF
        data[3] = v_uint & 0xFF
        data[4] = (kp_uint >> 8) & 0xFF
        data[5] = kp_uint & 0xFF
        data[6] = (kd_uint >> 8) & 0xFF
        data[7] = kd_uint & 0xFF

        msg = can.Message(arbitration_id=extid, data=data, is_extended_id=True)
        self._bus.send(msg)


# =============================================================================
# RSFollower Robot implementation
# =============================================================================


@dataclass(frozen=True)
class MotorSpec:
    side: str
    name: str
    model: str
    can_id: int

    @property
    def full_name(self) -> str:
        return f"{self.side}_{self.name}"

    @property
    def feature_key(self) -> str:
        return f"{self.full_name}.pos"


LEGACY_NAME_ALIASES: Dict[str, str] = {
    "wrist_flex": "wrist_pitch",
    "elbow_flex": "elbow_pitch",
    "shoulder_lift": "shoulder_roll",
    "shoulder_pan": "shoulder_pitch",
}

DEFAULT_RANGES_DEG: Dict[str, Tuple[float, float]] = {
    "gripper": (270.0, 340.0),
    "wrist_roll": (-45.0, 45.0),
    "wrist_pitch": (-45.0, 45.0),
    "wrist_yaw": (-45.0, 45.0),
    "elbow_pitch": (-45.0, 45.0),
    "shoulder_roll": (-45.0, 45.0),
    "shoulder_pitch": (-90.0, 90.0),
    "shoulder_yaw": (-90.0, 90.0),
}

ANGLE_PATTERN = re.compile(r"現在角度:\s*([0-9eE\+\-\.]+)\s*\[rad\]")


class RSFollower(Robot):
    """
    RobStride/QDD follower robot for 7DOF + Gripper arms.

    デフォルトの論理ジョイント:
        gripper, wrist_roll, wrist_pitch, wrist_yaw,
        elbow_pitch, shoulder_roll, shoulder_pitch, shoulder_yaw

    LEFT デフォルト ID:
        0x01..0x08
    RIGHT デフォルト ID:
        0x11..0x18
    """

    config_class = RSFollowerConfig
    name = "rs_follower"

    cfg: RSFollowerConfig

    def __init__(self, cfg: RSFollowerConfig):
        _apply_yaml_config_overrides(cfg)
        super().__init__(cfg)
        self.cfg = cfg

        self.cameras = make_cameras_from_configs(cfg.cameras)

        self._motor_specs: List[MotorSpec] = self._build_motor_specs()
        self._bus_by_motor: Dict[str, Any] = {}
        self._last_targets: Dict[str, Optional[float]] = {
            spec.full_name: None for spec in self._motor_specs
        }
        self._initial_position_ramp_pending: bool = bool(
            self.cfg.initial_position_ramp_enabled
        )
        self._initial_positions: Dict[str, float] = {}

        self._gripper_guard_lock = threading.Lock()
        self._gripper_feedback: Dict[str, Dict[str, Optional[float]]] = {}
        self._gripper_motor_status: Dict[str, MotorStatus] = {}
        self._gripper_guard_state: Dict[str, Dict[str, Any]] = {
            spec.full_name: {
                "last_limited_target": None,
                "last_command_time": None,
                "last_feedback_ts_seen": None,
                "stall_count": 0,
                "grasp_locked": False,
                "lock_target": None,
                "last_log_time": 0.0,
                "status_trip_count": 0,
                "current_trip_count": 0,
                "overcurrent_latched": False,
                "trip_time": 0.0,
                "trip_target": None,
                "hold_torque_nm": 0.0,
                "last_torque_update_time": None,
                "last_status_timestamp_seen": None,
                "last_current_timestamp_seen": None,
                "last_current_log_time": 0.0,
            }
            for spec in self._motor_specs
            if self._is_gripper(spec)
        }
        self._gripper_feedback_stop = threading.Event()
        self._gripper_feedback_thread: Optional[threading.Thread] = None

        self._ranges: Dict[str, Dict[str, float]] = self._build_default_ranges()

        self._is_connected: bool = False
        self._teleop_min: float = float(self.cfg.teleop_min)
        self._teleop_max: float = float(self.cfg.teleop_max)

        self._calib_path: Path = self._build_calibration_path()
        self._load_or_init_calibration()

    # ------------------------------------------------------------------ #
    # Motor spec / config helpers
    # ------------------------------------------------------------------ #
    def _normalise_list(self, values: Optional[Iterable[Any]]) -> List[Any]:
        if values is None:
            return []
        return list(values)

    def _ids_for_side(self, side: str, names_count: int) -> List[int]:
        if side == "left":
            explicit_ids = self._normalise_list(self.cfg.left_motor_ids)
            start_id = int(self.cfg.left_start_id)
            motor_count = int(self.cfg.left_motor_count)
        elif side == "right":
            explicit_ids = self._normalise_list(self.cfg.right_motor_ids)
            start_id = int(self.cfg.right_start_id)
            motor_count = int(self.cfg.right_motor_count)
        else:
            raise ValueError(f"Unknown side: {side}")

        if explicit_ids:
            ids = [int(v) for v in explicit_ids]
        else:
            ids = list(range(start_id, start_id + motor_count))

        if len(ids) != names_count:
            raise ValueError(
                f"{side}_motor_ids count mismatch: ids={len(ids)}, motor_names={names_count}. "
                f"IDs={ids}, motor_names={list(self.cfg.motor_names)}"
            )
        return ids

    def _build_motor_specs(self) -> List[MotorSpec]:
        motor_names = [str(v) for v in self._normalise_list(self.cfg.motor_names)]
        motor_models = [str(v).upper() for v in self._normalise_list(self.cfg.motor_models)]

        if not motor_names:
            motor_names = list(DEFAULT_MOTOR_NAMES)
        if not motor_models:
            motor_models = list(DEFAULT_MOTOR_MODELS)

        if len(motor_names) != len(motor_models):
            raise ValueError(
                "motor_names and motor_models must have the same length: "
                f"motor_names={motor_names}, motor_models={motor_models}"
            )

        specs: List[MotorSpec] = []
        for side in ("left", "right"):
            ids = self._ids_for_side(side, len(motor_names))
            for name, model, can_id in zip(motor_names, motor_models, ids):
                specs.append(
                    MotorSpec(
                        side=side,
                        name=str(name),
                        model=str(model).upper(),
                        can_id=int(can_id),
                    )
                )

        self._validate_unique_ids(specs)
        return specs

    def _validate_unique_ids(self, specs: List[MotorSpec]) -> None:
        seen: Dict[int, str] = {}
        for spec in specs:
            if spec.can_id in seen:
                raise ValueError(
                    f"Duplicate CAN ID 0x{spec.can_id:02X}: {seen[spec.can_id]} and {spec.full_name}. "
                    "左右を同一 CAN bus に接続する場合は ID が重複しないようにしてください。"
                )
            seen[spec.can_id] = spec.full_name

    def _iter_feature_names(self) -> List[str]:
        return [spec.feature_key for spec in self._motor_specs]

    def _get_mapping_value(
        self,
        mapping: Dict[str, float],
        full_name: str,
        short_name: str,
        default: float,
    ) -> float:
        if full_name in mapping:
            return float(mapping[full_name])
        if short_name in mapping:
            return float(mapping[short_name])
        return float(default)

    def _legacy_range_override(self, name: str, suffix: str) -> Optional[float]:
        legacy_by_new = {
            "gripper": "gripper",
            "wrist_roll": "wrist_roll",
            "wrist_pitch": "wrist_flex",
            "elbow_pitch": "elbow_flex",
            "shoulder_roll": "shoulder_lift",
            "shoulder_pitch": "shoulder_pan",
        }
        legacy_name = legacy_by_new.get(name)
        if legacy_name is None:
            return None
        attr = f"{legacy_name}_{suffix}_rad"
        return getattr(self.cfg, attr, None)

    def _build_default_ranges(self) -> Dict[str, Dict[str, float]]:
        ranges: Dict[str, Dict[str, float]] = {}
        for spec in self._motor_specs:
            open_deg, close_deg = DEFAULT_RANGES_DEG.get(spec.name, (-45.0, 45.0))
            default_open = math.radians(open_deg)
            default_close = math.radians(close_deg)

            legacy_open = self._legacy_range_override(spec.name, "open")
            legacy_close = self._legacy_range_override(spec.name, "close")
            if legacy_open is not None:
                default_open = float(legacy_open)
            if legacy_close is not None:
                default_close = float(legacy_close)

            ranges[spec.full_name] = {
                "open": self._get_mapping_value(
                    self.cfg.joint_open_rad,
                    spec.full_name,
                    spec.name,
                    default_open,
                ),
                "close": self._get_mapping_value(
                    self.cfg.joint_close_rad,
                    spec.full_name,
                    spec.name,
                    default_close,
                ),
            }
        return ranges

    def _is_inverted(self, spec: MotorSpec) -> bool:
        inverted = set(str(v) for v in self._normalise_list(self.cfg.inverted_motor_names))
        return spec.full_name in inverted or spec.name in inverted

    def _is_gripper(self, spec: MotorSpec) -> bool:
        gripper_names = set(
            str(v) for v in self._normalise_list(getattr(self.cfg, "gripper_motor_names", ["gripper"]))
        )
        if not gripper_names:
            return False
        return spec.full_name in gripper_names or spec.name in gripper_names

    def _gripper_specs(self) -> List[MotorSpec]:
        return [spec for spec in self._motor_specs if self._is_gripper(spec)]

    def _gains_for_motor(self, spec: MotorSpec) -> Tuple[float, float, float]:
        """Return kp/kd/t_ff for a motor.

        By default grippers use the same gains as the other joints, so their
        free-closing speed/response is unchanged.  Optional gripper-specific
        softer gains are applied only when gripper_soft_gains_enabled is true.
        """
        kp = float(self.cfg.kp)
        kd = float(self.cfg.kd)
        t_ff = float(self.cfg.t_ff)
        if self._is_gripper(spec) and bool(getattr(self.cfg, "gripper_soft_gains_enabled", False)):
            if self.cfg.gripper_kp is not None:
                kp = float(self.cfg.gripper_kp)
            if self.cfg.gripper_kd is not None:
                kd = float(self.cfg.gripper_kd)
            if self.cfg.gripper_t_ff is not None:
                t_ff = float(self.cfg.gripper_t_ff)
        return kp, kd, t_ff

    def _create_bus(self, spec: MotorSpec) -> Any:
        # RS06 は既存の専用レンジ/送信ループ実装を使います。
        # RS00/RS03/RS05 は RobStrideBus 経由で送信します。
        kp, kd, t_ff = self._gains_for_motor(spec)
        if spec.model.upper() == "RS06":
            return RS06Bus(
                channel=self.cfg.channel,
                motor_id=spec.can_id,
                kp=kp,
                kd=kd,
                default_v_set=self.cfg.default_v_set,
                t_ff=t_ff,
                tx_hz=self.cfg.tx_hz,
            )
        is_gripper = self._is_gripper(spec)
        torque_limit_nm: Optional[float] = None
        if is_gripper and bool(getattr(self.cfg, "gripper_hardware_torque_limit_enabled", True)):
            torque_limit_nm = float(getattr(self.cfg, "gripper_max_torque_nm", 3.0))
        joint_limit_configured = False
        if not is_gripper:
            joint_limits = getattr(self.cfg, "joint_torque_limits_nm", None) or {}
            if spec.full_name in joint_limits:
                torque_limit_nm = float(joint_limits[spec.full_name])
                joint_limit_configured = True
                logger.info(
                    "RSFollower: %s (0x%02X) のモータ側トルク上限を %.2f N.m に設定します",
                    spec.full_name, spec.can_id, torque_limit_nm,
                )

        current_limit_a: Optional[float] = None
        if is_gripper:
            configured_current_limit = getattr(
                self.cfg,
                "gripper_hardware_current_limit_a",
                None,
            )
            if configured_current_limit is not None:
                current_limit_a = float(configured_current_limit)

        current_monitor_hz = (
            max(0.0, float(getattr(self.cfg, "gripper_current_monitor_hz", 0.0)))
            if is_gripper
            else 0.0
        )
        monitor_feedback = bool(
            is_gripper
            and (
                bool(getattr(self.cfg, "gripper_status_guard_enabled", True))
                or torque_limit_nm is not None
                or current_limit_a is not None
                or current_monitor_hz > 0.0
            )
        )

        return RobStrideBus(
            channel=self.cfg.channel,
            motor_id=spec.can_id,
            kp=kp,
            kd=kd,
            default_v_set=self.cfg.default_v_set,
            t_ff=t_ff,
            tx_hz=self.cfg.tx_hz,
            model=spec.model,
            monitor_feedback=monitor_feedback,
            feedback_stale_s=max(
                0.0,
                float(getattr(self.cfg, "gripper_status_stale_s", 0.25)),
            ),
            hardware_torque_limit_nm=torque_limit_nm,
            hardware_torque_limit_required=(
                (is_gripper and bool(getattr(self.cfg, "gripper_torque_limit_required", True)))
                or (joint_limit_configured
                    and bool(getattr(self.cfg, "joint_torque_limit_required", True)))
            ),
            hardware_torque_limit_verify=bool(
                getattr(self.cfg, "gripper_torque_limit_verify", True)
            ),
            hardware_current_limit_a=current_limit_a,
            hardware_current_limit_required=(
                is_gripper
                and bool(getattr(self.cfg, "gripper_current_limit_required", True))
            ),
            hardware_current_limit_verify=bool(
                getattr(self.cfg, "gripper_current_limit_verify", True)
            ),
            current_monitor_hz=current_monitor_hz,
        )

    # ------------------------------------------------------------------ #
    # Robot 基底クラス: MotorCalibration は使わない
    # ------------------------------------------------------------------ #
    def _load_calibration(self) -> None:  # type: ignore[override]
        self.calibration = {}

    # ------------------------------------------------------------------ #
    # Robot 抽象メソッド
    # ------------------------------------------------------------------ #
    def configure(self) -> None:  # type: ignore[override]
        logger.info("%s RSFollower.configure(): no-op", self.cfg.id)

    @property
    def action_features(self) -> Dict[str, Any]:
        return {feature_name: float for feature_name in self._iter_feature_names()}

    @property
    def observation_features(self) -> Dict[str, Any]:
        feats: Dict[str, Any] = {
            feature_name: float for feature_name in self._iter_feature_names()
        }

        for name, cam in self.cameras.items():
            feats[name] = (cam.height, cam.width, 3)

        return feats

    @property
    def is_connected(self) -> bool:  # type: ignore[override]
        cams_ok = all(cam.is_connected for cam in getattr(self, "cameras", {}).values())
        return self._is_connected and cams_ok

    @property
    def is_calibrated(self) -> bool:  # type: ignore[override]
        return self._calib_path.is_file()

    # ------------------------------------------------------------------ #
    # キャリブファイル
    # ------------------------------------------------------------------ #
    def _build_calibration_path(self) -> Path:
        if self.cfg.calibration_dir is not None:
            root = Path(self.cfg.calibration_dir).expanduser()
        else:
            root = Path.home() / ".cache" / "huggingface" / "lerobot"

        calib_dir = root / self.cfg.rs_calibration_subdir
        calib_dir.mkdir(parents=True, exist_ok=True)
        return calib_dir / f"{self.cfg.id}.json"

    def _load_or_init_calibration(self) -> None:
        data: Dict[str, Any] = {}
        if self._calib_path.is_file():
            try:
                with self._calib_path.open("r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception:
                logger.exception("Failed to load calibration from %s", self._calib_path)
                data = {}

        if data:
            ranges_data = data.get("ranges")
            if isinstance(ranges_data, dict):
                for full_name, values in ranges_data.items():
                    if not isinstance(values, dict):
                        continue
                    if full_name not in self._ranges:
                        continue
                    open_rad = values.get("open_rad")
                    close_rad = values.get("close_rad")
                    if open_rad is not None and close_rad is not None:
                        self._ranges[full_name] = {
                            "open": float(open_rad),
                            "close": float(close_rad),
                        }
            else:
                self._load_legacy_calibration(data)

            self._teleop_min = float(data.get("teleop_min", self._teleop_min))
            self._teleop_max = float(data.get("teleop_max", self._teleop_max))

            logger.info(
                "RSFollower: loaded calibration from %s\n%s",
                self._calib_path,
                self._format_ranges_for_log(),
            )
        else:
            logger.info(
                "RSFollower: no calibration file found for id=%s. "
                "Will run calibrate() to measure actual joint angles.",
                self.cfg.id,
            )

    def _load_legacy_calibration(self, data: Dict[str, Any]) -> None:
        legacy_to_new = {
            "gripper": "gripper",
            "wrist_roll": "wrist_roll",
            "wrist_flex": "wrist_pitch",
            "elbow_flex": "elbow_pitch",
            "shoulder_lift": "shoulder_roll",
            "shoulder_pan": "shoulder_pitch",
        }
        for side in ("left", "right"):
            for legacy_name, new_name in legacy_to_new.items():
                full_name = f"{side}_{new_name}"
                if full_name not in self._ranges:
                    continue
                open_key = f"{side}_{legacy_name}_open_rad"
                close_key = f"{side}_{legacy_name}_close_rad"
                open_rad = data.get(open_key)
                close_rad = data.get(close_key)
                if open_rad is not None and close_rad is not None:
                    self._ranges[full_name] = {
                        "open": float(open_rad),
                        "close": float(close_rad),
                    }

    def _calibration_payload(self) -> Dict[str, Any]:
        motors = {
            spec.full_name: {
                "can_id": spec.can_id,
                "can_id_hex": f"0x{spec.can_id:02X}",
                "model": spec.model,
            }
            for spec in self._motor_specs
        }
        ranges = {
            full_name: {
                "open_rad": values["open"],
                "close_rad": values["close"],
            }
            for full_name, values in self._ranges.items()
        }
        return {
            "version": 20,
            "id": self.cfg.id,
            "channel": self.cfg.channel,
            "motors": motors,
            "ranges": ranges,
            "teleop_min": self._teleop_min,
            "teleop_max": self._teleop_max,
        }

    def _save_calibration(self) -> None:
        data = self._calibration_payload()
        try:
            self._calib_path.parent.mkdir(parents=True, exist_ok=True)
            with self._calib_path.open("w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception:
            logger.exception("Failed to write calibration file %s", self._calib_path)
            return

        logger.info(
            "RSFollower: saved calibration to %s\n%s",
            self._calib_path,
            self._format_ranges_for_log(),
        )

    def _format_ranges_for_log(self) -> str:
        lines = []
        for spec in self._motor_specs:
            values = self._ranges[spec.full_name]
            lines.append(
                "  {name:24s} ID=0x{id:02X} {model:4s} "
                "open={open_rad:.6f} rad ({open_deg:.2f} deg), "
                "close={close_rad:.6f} rad ({close_deg:.2f} deg)".format(
                    name=spec.full_name,
                    id=spec.can_id,
                    model=spec.model,
                    open_rad=values["open"],
                    open_deg=math.degrees(values["open"]),
                    close_rad=values["close"],
                    close_deg=math.degrees(values["close"]),
                )
            )
        return "\n".join(lines)

    # ------------------------------------------------------------------ #
    # 起動時の安全な初期位置移動
    # ------------------------------------------------------------------ #
    def _get_angle_script_path(self) -> Path:
        script = getattr(self.cfg, "get_angle_script", None)
        if script:
            return Path(str(script)).expanduser()
        return Path.home() / "RS" / "get_angle.py"

    def _read_angle_once(
        self,
        motor_id: int,
        timeout: Optional[float] = None,
        quiet: bool = False,
    ) -> Optional[float]:
        script_path = self._get_angle_script_path()
        if not script_path.is_file():
            if not quiet:
                logger.warning("get_angle.py が見つかりません: %s", script_path)
            return None

        cmd = [
            sys.executable,
            str(script_path),
            "--channel",
            self.cfg.channel,
            "--id",
            f"0x{motor_id:02X}",
            "--timeout",
            str(timeout if timeout is not None else self.cfg.initial_position_read_timeout_s),
        ]
        try:
            proc = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                check=False,
            )
        except Exception as e:
            if quiet:
                logger.debug("Failed to run get_angle.py (id=0x%02X): %s", motor_id, e)
            else:
                logger.error("Failed to run get_angle.py (id=0x%02X): %s", motor_id, e)
            return None

        out = proc.stdout + proc.stderr
        logger.debug("get_angle.py output for id=0x%02X:\n%s", motor_id, out)

        match = ANGLE_PATTERN.search(out)
        if not match:
            if quiet:
                logger.debug(
                    "No angle found in get_angle.py output for ID 0x%02X. Output was:\n%s",
                    motor_id,
                    out,
                )
            else:
                logger.warning(
                    "No angle found in get_angle.py output for ID 0x%02X. Output was:\n%s",
                    motor_id,
                    out,
                )
            return None

        try:
            return float(match.group(1))
        except ValueError:
            if not quiet:
                logger.warning(
                    "Failed to parse angle from output for ID 0x%02X: %s",
                    motor_id,
                    match.group(1),
                )
            return None

    def _prepare_initial_position_ramp(self) -> None:
        """Read actual joint angles before enabling motors.

        The first target command after startup can otherwise jump directly to the
        first action value.  By recording the current angle here, the first
        send_action() ramps from the measured pose to the requested pose.
        """
        if not self.cfg.initial_position_ramp_enabled:
            self._initial_position_ramp_pending = False
            self._initial_positions = {}
            return

        self._initial_position_ramp_pending = True
        self._initial_positions = {}
        missing: List[str] = []
        retries = max(1, int(self.cfg.initial_position_read_retries))
        timeout = float(self.cfg.initial_position_read_timeout_s)

        for spec in self._motor_specs:
            angle: Optional[float] = None
            for _ in range(retries):
                angle = self._read_angle_once(spec.can_id, timeout=timeout)
                if angle is not None:
                    break

            if angle is None:
                missing.append(f"{spec.full_name}(0x{spec.can_id:02X})")
                # require_feedback=False のときだけ、従来と同じ open 側を暫定開始点にします。
                # 実機の現在角度と異なる可能性があるため、デフォルトでは使用しません。
                self._initial_positions[spec.full_name] = self._ranges[spec.full_name]["open"]
            else:
                self._initial_positions[spec.full_name] = float(angle)
                self._last_targets[spec.full_name] = float(angle)
                if self._is_gripper(spec):
                    self._update_gripper_feedback(spec.full_name, float(angle))

        if missing:
            message = (
                "Initial-position ramp could not read current angle for: "
                + ", ".join(missing)
                + ". 起動直後の急動作を防ぐため、get_angle.py と CAN 接続を確認してください。"
            )
            if self.cfg.initial_position_require_feedback:
                raise RuntimeError(message)
            logger.warning("%s Falling back to calibration/open positions.", message)

        logger.info(
            "RSFollower: initial-position ramp prepared from current angles:\n%s",
            "\n".join(
                f"  {spec.full_name:24s} ID=0x{spec.can_id:02X} "
                f"start={self._initial_positions[spec.full_name]:.6f} rad"
                for spec in self._motor_specs
            ),
        )

    def _write_goal_positions(self, targets: Dict[str, float]) -> None:
        for spec in self._motor_specs:
            self._bus_by_motor[spec.full_name].sync_write(
                "Goal_Position", {"qdd0": targets[spec.full_name]}
            )

    def _move_to_initial_position(self) -> None:
        """connect() 内で cfg.initial_position (teleop 単位) へブロッキング移動する。

        完了後に connect() が返るため、制御ループ (推論/teleop) はアームが
        初期位置で静止した状態から始まる。移動には既存のランプ機構を使い、
        消化された pending フラグにより以後の send_action は即時実行になる。
        """
        unknown = [k for k in self.cfg.initial_position if k not in self._ranges]
        if unknown:
            raise ValueError(
                f"initial_position に未知の関節名があります: {unknown}. "
                f"有効な関節名: {sorted(self._ranges)}"
            )
        if not (self.cfg.initial_position_ramp_enabled and self._initial_position_ramp_pending):
            logger.warning(
                "initial_position が設定されていますが、ランプ機構が無効のため"
                "急動作防止の観点から移動をスキップします "
                "(initial_position_ramp_enabled=True にしてください)"
            )
            return

        targets = self._initial_position_targets()

        logger.info(
            "RSFollower: connect() 内で初期位置へ移動します (完了までブロック): %s",
            ", ".join(f"{k}={v:.1f}" for k, v in self.cfg.initial_position.items()),
        )
        self._run_initial_position_ramp(targets)
        logger.info("RSFollower: 初期位置への移動完了 — 制御ループを開始できます")

    def _initial_position_targets(self) -> Dict[str, float]:
        """cfg.initial_position (teleop 単位) から各関節の目標角 [rad] を作る。"""
        targets: Dict[str, float] = {}
        for spec in self._motor_specs:
            name = spec.full_name
            if name in self.cfg.initial_position:
                values = self._ranges[name]
                targets[name] = self._teleop_to_rad(
                    float(self.cfg.initial_position[name]),
                    values["open"],
                    values["close"],
                    None,  # 初回移動なので max_relative_target のクランプは不要
                )
            else:
                # 未指定の関節は現在角度を維持
                start = self._initial_positions.get(name)
                if start is None:
                    start = self._last_targets.get(name)
                targets[name] = float(start) if start is not None else self._ranges[name]["open"]
        return targets

    def _return_to_initial_position(self) -> None:
        """disconnect 前に初期位置へゆっくり戻る (収録/推論終了後の後片付け)。

        connect 時と同じランプ機構を使う。現在角度は実フィードバックを再取得
        するため、テレオペ終了時のどんな姿勢からでも滑らかに戻る。失敗しても
        disconnect 自体は続行する (CAN 断などの異常系でハングさせない)。
        """
        if not (self.cfg.return_to_initial_on_disconnect and self.cfg.initial_position):
            return
        if not self.cfg.initial_position_ramp_enabled:
            return
        try:
            self._prepare_initial_position_ramp()
            if not self._initial_position_ramp_pending:
                logger.warning("RSFollower: 現在角度が読めないため初期位置への復帰をスキップ")
                return
            logger.info("RSFollower: 収録終了 — 初期位置へゆっくり戻ります")
            self._run_initial_position_ramp(self._initial_position_targets())
            logger.info("RSFollower: 初期位置への復帰完了")
        except Exception:
            logger.exception("RSFollower: 初期位置への復帰に失敗 (disconnect は続行)")

    def _run_initial_position_ramp(self, targets: Dict[str, float]) -> None:
        if not self.cfg.initial_position_ramp_enabled or not self._initial_position_ramp_pending:
            self._write_goal_positions(targets)
            return

        starts: Dict[str, float] = {}
        for spec in self._motor_specs:
            full_name = spec.full_name
            start = self._initial_positions.get(full_name)
            if start is None:
                last = self._last_targets.get(full_name)
                start = last if last is not None else self._ranges[full_name]["open"]
            starts[full_name] = float(start)

        # 多回転座標系ずれの正規化 (2026-08-27 の 0x16 全周回転・断線事故の再発防止):
        # RobStride の位置は ±4π の多回転絶対値。電源サイクル後は同じ物理姿勢が
        # ±2π ずれて報告され得るため、目標を現在角と同じ回転周の最近傍表現に写す。
        if self.cfg.initial_position_wrap_normalize:
            two_pi = 2.0 * math.pi
            for spec in self._motor_specs:
                name = spec.full_name
                turns = round((starts[name] - targets[name]) / two_pi)
                if turns != 0:
                    logger.warning(
                        "RSFollower: %s の目標を %+d 回転分正規化しました "
                        "(座標系ずれ検出: start=%.3f target=%.3f → %.3f rad)。"
                        "較正と現在角の座標系が一致していません — 再キャリブレーション推奨",
                        name, turns, starts[name], targets[name],
                        targets[name] + turns * two_pi,
                    )
                    targets[name] = targets[name] + turns * two_pi

        max_delta = max(
            (abs(targets[spec.full_name] - starts[spec.full_name]) for spec in self._motor_specs),
            default=0.0,
        )

        # 正規化後も大移動が必要なら座標系異常のサイン → 一切動かずに中止
        max_travel = float(self.cfg.initial_position_max_travel_rad)
        if max_travel > 0.0 and max_delta > max_travel:
            worst = max(
                self._motor_specs,
                key=lambda s: abs(targets[s.full_name] - starts[s.full_name]),
            )
            raise RuntimeError(
                f"RSFollower: 初期位置ランプの移動量 {max_delta:.2f} rad が上限 "
                f"{max_travel:.2f} rad を超過 (最大: {worst.full_name} "
                f"{starts[worst.full_name]:.3f}→{targets[worst.full_name]:.3f} rad)。"
                "較正と現在角の座標系ずれの疑いがあるため動作を中止しました。"
                "現在角の確認と再キャリブレーションを行ってください "
                "(initial_position_max_travel_rad で上限変更可)"
            )

        # すでに目標付近にいる場合は 5 秒のランプを省略して即書き込み
        if max_delta <= float(self.cfg.initial_position_ramp_skip_within_rad):
            logger.info(
                "RSFollower: 目標との差 %.3f rad ≤ %.3f rad — ランプ省略",
                max_delta, self.cfg.initial_position_ramp_skip_within_rad,
            )
            self._write_goal_positions(targets)
            for full_name, value in targets.items():
                self._last_targets[full_name] = value
            self._initial_position_ramp_pending = False
            self._initial_positions = {}
            return

        interval_cfg = self.cfg.initial_position_ramp_interval_s
        if interval_cfg is None:
            interval = 1.0 / float(self.cfg.tx_hz) if self.cfg.tx_hz > 0 else 0.02
        else:
            interval = float(interval_cfg)
        interval = max(0.005, interval)

        duration = max(0.0, float(self.cfg.initial_position_ramp_duration_s))
        max_speed = float(self.cfg.initial_position_ramp_max_speed_rad_s)
        if max_speed > 0.0 and max_delta > 0.0:
            duration = max(duration, max_delta / max_speed)

        steps = max(1, int(math.ceil(duration / interval)))

        logger.info(
            "RSFollower: moving to initial target slowly: max_delta=%.3f rad, "
            "duration=%.2f s, steps=%d, interval=%.3f s",
            max_delta,
            duration,
            steps,
            interval,
        )

        # 最初の 1 フレームは現在角度を目標値にして、急な位置誤差を作らない。
        self._write_goal_positions(starts)
        for full_name, value in starts.items():
            self._last_targets[full_name] = value
        if steps > 1:
            time.sleep(interval)

        for step in range(1, steps + 1):
            alpha = step / steps
            intermediate = {
                spec.full_name: starts[spec.full_name]
                + (targets[spec.full_name] - starts[spec.full_name]) * alpha
                for spec in self._motor_specs
            }
            self._write_goal_positions(intermediate)
            for full_name, value in intermediate.items():
                self._last_targets[full_name] = value
            if step < steps:
                time.sleep(interval)

        self._initial_position_ramp_pending = False
        self._initial_positions = {}

    # ------------------------------------------------------------------ #
    # グリッパ トルク / 過電流ガード
    # ------------------------------------------------------------------ #
    def _update_gripper_feedback(self, full_name: str, position_rad: float) -> None:
        now = time.monotonic()
        with self._gripper_guard_lock:
            prev = self._gripper_feedback.get(full_name, {})
            self._gripper_feedback[full_name] = {
                "position": float(position_rad),
                "timestamp": now,
                "prev_position": prev.get("position"),
                "prev_timestamp": prev.get("timestamp"),
            }

    def _get_gripper_feedback(
        self,
        full_name: str,
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """Return ``(position, age, movement, sample_timestamp)``."""
        now = time.monotonic()
        with self._gripper_guard_lock:
            data = dict(self._gripper_feedback.get(full_name, {}))

        pos = data.get("position")
        ts = data.get("timestamp")
        if pos is None or ts is None:
            return None, None, None, None

        prev_pos = data.get("prev_position")
        movement: Optional[float] = None
        if prev_pos is not None:
            movement = abs(float(pos) - float(prev_pos))
        return float(pos), now - float(ts), movement, float(ts)

    def _refresh_gripper_motor_status(self) -> None:
        """Copy live type-2 motor feedback from each gripper bus.

        The bus receives status frames in its own background thread.  This
        method is non-blocking and only snapshots the newest value for the
        control loop.  Position from the same frame also feeds the existing
        contact/stall detector, with ``get_angle.py`` retained as a fallback.
        """
        if not bool(getattr(self.cfg, "gripper_status_guard_enabled", True)):
            return

        for spec in self._gripper_specs():
            bus = self._bus_by_motor.get(spec.full_name)
            getter = getattr(bus, "get_latest_status", None)
            if not callable(getter):
                continue
            try:
                status = getter()
            except Exception:
                logger.debug(
                    "Failed to snapshot gripper motor status for %s",
                    spec.full_name,
                    exc_info=True,
                )
                continue
            if status is None:
                continue

            should_update_position = False
            with self._gripper_guard_lock:
                previous = self._gripper_motor_status.get(spec.full_name)
                status_changed = previous is None or previous.timestamp != status.timestamp
                current_changed = (
                    previous is None
                    or previous.current_timestamp != status.current_timestamp
                )
                if status_changed or current_changed:
                    self._gripper_motor_status[spec.full_name] = status
                    should_update_position = status_changed
            if should_update_position:
                self._update_gripper_feedback(spec.full_name, float(status.position))

    def _get_gripper_motor_status(
        self,
        full_name: str,
    ) -> Tuple[Optional[MotorStatus], Optional[float]]:
        now = time.monotonic()
        with self._gripper_guard_lock:
            status = self._gripper_motor_status.get(full_name)
        if status is None:
            return None, None
        return status, max(0.0, now - float(status.timestamp))

    def _start_gripper_guard_feedback(self) -> None:
        if not self.cfg.gripper_overcurrent_guard_enabled:
            return

        # The type-2 status listener is owned by each RobStrideBus.  Snapshot it
        # immediately; the external angle reader below remains a compatibility
        # fallback for firmware/adapters that do not expose status frames.
        self._refresh_gripper_motor_status()

        if not self.cfg.gripper_guard_feedback_enabled:
            return
        if self._gripper_feedback_thread is not None and self._gripper_feedback_thread.is_alive():
            return

        specs = self._gripper_specs()
        if not specs:
            return

        self._gripper_feedback_stop.clear()
        self._gripper_feedback_thread = threading.Thread(
            target=self._gripper_feedback_loop,
            name="RSFollowerGripperGuardFeedback",
            daemon=True,
        )
        self._gripper_feedback_thread.start()
        logger.info(
            "RSFollower: gripper guard started for %s; angle fallback %.1f Hz; "
            "max torque %.2f N.m",
            ", ".join(spec.full_name for spec in specs),
            float(self.cfg.gripper_guard_feedback_hz),
            float(getattr(self.cfg, "gripper_max_torque_nm", 3.0)),
        )

    def _stop_gripper_guard_feedback(self) -> None:
        self._gripper_feedback_stop.set()
        thread = self._gripper_feedback_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._gripper_feedback_thread = None

    def _gripper_feedback_loop(self) -> None:
        specs = self._gripper_specs()
        hz = max(0.1, float(self.cfg.gripper_guard_feedback_hz))
        period = 1.0 / hz
        timeout = max(0.001, float(self.cfg.gripper_guard_feedback_timeout_s))

        while not self._gripper_feedback_stop.is_set():
            started = time.monotonic()
            self._refresh_gripper_motor_status()
            for spec in specs:
                if self._gripper_feedback_stop.is_set():
                    break

                # Type-2 feedback already contains the gripper position.  Do
                # not spawn get_angle.py while that direct feedback is fresh;
                # the external reader is only a compatibility fallback.
                status, status_age = self._get_gripper_motor_status(spec.full_name)
                if (
                    status is not None
                    and status_age is not None
                    and status_age <= max(
                        0.0,
                        float(getattr(self.cfg, "gripper_status_stale_s", 0.25)),
                    )
                ):
                    continue

                angle = self._read_angle_once(spec.can_id, timeout=timeout, quiet=True)
                if angle is not None:
                    self._update_gripper_feedback(spec.full_name, float(angle))

            elapsed = time.monotonic() - started
            self._gripper_feedback_stop.wait(max(0.0, period - elapsed))

    def _log_gripper_guard(self, spec: MotorSpec, level: int, msg: str, *args: Any) -> None:
        state = self._gripper_guard_state.setdefault(spec.full_name, {})
        now = time.monotonic()
        last_log = float(state.get("last_log_time") or 0.0)
        interval = max(0.0, float(self.cfg.gripper_guard_log_interval_s))
        if now - last_log >= interval:
            logger.log(level, "RSFollower gripper guard %s: " + msg, spec.full_name, *args)
            state["last_log_time"] = now

    @staticmethod
    def _limit_along_closing_direction(
        value: float,
        reference: float,
        closing_dir: float,
        max_closing_delta: float,
        max_opening_delta: float,
    ) -> float:
        delta = (float(value) - float(reference)) * closing_dir
        if max_closing_delta >= 0.0 and delta > max_closing_delta:
            return float(reference) + closing_dir * max_closing_delta
        if max_opening_delta >= 0.0 and delta < -max_opening_delta:
            return float(reference) - closing_dir * max_opening_delta
        return float(value)

    def _position_error_for_gripper_torque(
        self,
        spec: MotorSpec,
        torque_nm: float,
    ) -> float:
        """Convert a nominal stationary torque target to bounded PD error.

        In MIT impedance control, the stationary proportional contribution is
        approximately ``Kp * position_error``.  The motor-side ``limit_torque``
        remains the authoritative hard limit; this conversion is a second,
        software-side bound used after contact.
        """
        kp, _, t_ff = self._gains_for_motor(spec)
        available = max(0.0, float(torque_nm) - abs(float(t_ff)))
        if kp <= 1e-9:
            return 0.0
        error = available / kp
        absolute_cap = max(
            0.0,
            float(getattr(self.cfg, "gripper_guard_max_hold_error_rad", 0.0)),
        )
        if absolute_cap > 0.0:
            error = min(error, absolute_cap)
        return max(0.0, error)

    @staticmethod
    def _default_gripper_state() -> Dict[str, Any]:
        return {
            "last_limited_target": None,
            "last_command_time": None,
            "last_feedback_ts_seen": None,
            "stall_count": 0,
            "grasp_locked": False,
            "lock_target": None,
            "last_log_time": 0.0,
            "status_trip_count": 0,
            "current_trip_count": 0,
            "overcurrent_latched": False,
            "trip_time": 0.0,
            "trip_target": None,
            "hold_torque_nm": 0.0,
            "last_torque_update_time": None,
            "last_status_timestamp_seen": None,
            "last_current_timestamp_seen": None,
            "last_current_log_time": 0.0,
        }

    def _apply_gripper_overcurrent_guard(
        self,
        spec: MotorSpec,
        requested_target: float,
        teleop_value: float,
    ) -> float:
        """Limit grip torque and react to RobStride over-current feedback.

        Free-motion position commands are not rate-limited.  The RS05 firmware
        is configured with ``limit_torque = gripper_max_torque_nm`` before the
        motor is enabled.  Once contact is detected, this method additionally
        controls the position-error window from a nominal torque target,
        monitors reported torque, and backs off/latches on hard torque or
        over-current/fault flags.
        """
        if not self.cfg.gripper_overcurrent_guard_enabled or not self._is_gripper(spec):
            return float(requested_target)

        full_name = spec.full_name
        state = self._gripper_guard_state.setdefault(
            full_name,
            self._default_gripper_state(),
        )

        values = self._ranges[full_name]
        open_rad = float(values["open"])
        close_rad = float(values["close"])
        closing_dir = 1.0 if close_rad >= open_rad else -1.0

        now = time.monotonic()
        last_limited = state.get("last_limited_target")
        if last_limited is None:
            last_known = self._last_targets.get(full_name)
            last_limited = float(last_known) if last_known is not None else float(requested_target)
        last_limited = float(last_limited)
        target = float(requested_target)

        pos, age, movement, sample_ts = self._get_gripper_feedback(full_name)
        feedback_ok = (
            pos is not None
            and age is not None
            and age <= max(0.0, float(self.cfg.gripper_guard_feedback_stale_s))
        )

        status, status_age = self._get_gripper_motor_status(full_name)
        status_ok = (
            bool(getattr(self.cfg, "gripper_status_guard_enabled", True))
            and status is not None
            and status_age is not None
            and status_age <= max(
                0.0,
                float(getattr(self.cfg, "gripper_status_stale_s", 0.25)),
            )
        )
        measured_torque = abs(float(status.torque)) if status_ok and status is not None else None
        measured_current: Optional[float] = None
        current_age: Optional[float] = None
        current_ok = False
        if (
            status is not None
            and status.current_a is not None
            and status.current_timestamp is not None
        ):
            current_age = max(0.0, now - float(status.current_timestamp))
            current_ok = current_age <= max(
                0.0,
                float(getattr(self.cfg, "gripper_current_stale_s", 0.35)),
            )
            if current_ok:
                measured_current = abs(float(status.current_a))

        if (
            measured_current is not None
            and bool(getattr(self.cfg, "gripper_current_log_enabled", False))
        ):
            last_current_log = float(state.get("last_current_log_time") or 0.0)
            current_log_interval = max(
                0.0,
                float(getattr(self.cfg, "gripper_current_log_interval_s", 1.0)),
            )
            if now - last_current_log >= current_log_interval:
                logger.info(
                    "RSFollower gripper telemetry %s: current=%.3f A, "
                    "torque=%s N.m, temperature=%s C, status_age=%s s, "
                    "current_age=%s s",
                    full_name,
                    measured_current,
                    f"{measured_torque:.3f}" if measured_torque is not None else "n/a",
                    (
                        f"{float(status.temperature):.1f}"
                        if status is not None
                        else "n/a"
                    ),
                    f"{status_age:.3f}" if status_age is not None else "n/a",
                    f"{current_age:.3f}" if current_age is not None else "n/a",
                )
                state["last_current_log_time"] = now

        def finish(value: float) -> float:
            state["last_limited_target"] = float(value)
            state["last_command_time"] = now
            state["last_teleop_value"] = float(teleop_value)
            return float(value)

        def clamp_additional_close(value: float, limit: float) -> float:
            if (float(value) - float(limit)) * closing_dir > 0.0:
                return float(limit)
            return float(value)

        opening_requested = (target - last_limited) * closing_dir < -1e-6

        # If the independent angle feedback is stale, type-2 position may still
        # have refreshed it in _refresh_gripper_motor_status().  When neither is
        # available and require_feedback=true, never add more closing error.
        if not feedback_ok:
            if self.cfg.gripper_guard_feedback_enabled and self.cfg.gripper_guard_require_feedback:
                if not opening_requested and (target - last_limited) * closing_dir > 0.0:
                    target = last_limited
                self._log_gripper_guard(
                    spec,
                    logging.WARNING,
                    "position feedback unavailable/stale; blocking additional close at %.4f rad",
                    target,
                )
            return finish(target)

        assert pos is not None

        max_torque = max(0.0, float(getattr(self.cfg, "gripper_max_torque_nm", 3.0)))
        hard_limit = max(
            0.0,
            float(getattr(self.cfg, "gripper_torque_hard_limit_nm", max_torque)),
        )
        if hard_limit <= 0.0:
            hard_limit = max_torque
        # The motor-side limit is verified before enable.  Permit a small
        # software hard-trip margin above it so normal saturation at exactly
        # 3.0 N.m does not look like an over-torque event.  Never allow a
        # configured RS05 threshold above its documented 5.5 N.m peak scale.
        if spec.model.upper() == "RS05" and hard_limit > 0.0:
            hard_limit = min(hard_limit, 5.5)
        soft_limit = max(
            0.0,
            float(getattr(self.cfg, "gripper_torque_soft_limit_nm", max_torque)),
        )
        if max_torque > 0.0:
            soft_limit = min(soft_limit, max_torque)
        if hard_limit > 0.0:
            soft_limit = min(soft_limit, hard_limit)
        release_torque = max(
            0.0,
            float(getattr(self.cfg, "gripper_torque_release_nm", soft_limit)),
        )
        if soft_limit > 0.0:
            release_torque = min(release_torque, soft_limit)

        def optional_positive_config(name: str) -> Optional[float]:
            value = getattr(self.cfg, name, None)
            if value is None:
                return None
            parsed = float(value)
            return parsed if parsed > 0.0 else None

        current_soft_limit = optional_positive_config("gripper_current_soft_limit_a")
        current_hard_limit = optional_positive_config("gripper_current_hard_limit_a")
        if current_soft_limit is not None and current_hard_limit is not None:
            current_soft_limit = min(current_soft_limit, current_hard_limit)
        current_release = optional_positive_config("gripper_current_release_a")
        if current_release is None:
            current_release = (
                0.80 * current_soft_limit
                if current_soft_limit is not None
                else (0.75 * current_hard_limit if current_hard_limit is not None else None)
            )
        if current_soft_limit is not None and current_release is not None:
            current_release = min(current_release, current_soft_limit)

        hard_status_fault = bool(status.hard_fault) if status_ok and status is not None else False
        direct_overcurrent = bool(
            status_ok
            and status is not None
            and (status.overcurrent or status.fault_stall_current)
        )

        torque_hard = (
            measured_torque is not None
            and hard_limit > 0.0
            and measured_torque >= hard_limit
        )

        # Count only unique motor status/current samples.  The LeRobot control
        # loop can run faster than CAN feedback, and counting the same frame
        # twice would make a two-sample trip effectively a one-sample trip.
        new_status_sample = bool(
            status_ok
            and status is not None
            and status.timestamp != state.get("last_status_timestamp_seen")
        )
        if new_status_sample and status is not None:
            state["last_status_timestamp_seen"] = status.timestamp
            if torque_hard:
                state["status_trip_count"] = int(state.get("status_trip_count") or 0) + 1
            elif not direct_overcurrent and not hard_status_fault:
                state["status_trip_count"] = 0

        current_hard = bool(
            measured_current is not None
            and current_hard_limit is not None
            and measured_current >= current_hard_limit
        )
        new_current_sample = bool(
            current_ok
            and status is not None
            and status.current_timestamp is not None
            and status.current_timestamp != state.get("last_current_timestamp_seen")
        )
        if new_current_sample and status is not None:
            state["last_current_timestamp_seen"] = status.current_timestamp
            if current_hard:
                state["current_trip_count"] = int(state.get("current_trip_count") or 0) + 1
            else:
                state["current_trip_count"] = 0

        trip_confirm = max(
            1,
            int(getattr(self.cfg, "gripper_torque_trip_confirm_count", 2)),
        )
        current_trip_confirm = max(
            1,
            int(getattr(self.cfg, "gripper_current_trip_confirm_count", 2)),
        )
        should_trip = bool(
            direct_overcurrent
            or hard_status_fault
            or int(state.get("status_trip_count") or 0) >= trip_confirm
            or int(state.get("current_trip_count") or 0) >= current_trip_confirm
        )

        if should_trip:
            backoff = max(
                0.0,
                float(getattr(self.cfg, "gripper_overcurrent_backoff_rad", 0.04)),
            )
            recovery_target = pos - closing_dir * backoff
            state["overcurrent_latched"] = True
            state["trip_time"] = now
            state["trip_target"] = recovery_target
            state["grasp_locked"] = True
            state["lock_target"] = recovery_target
            state["hold_torque_nm"] = 0.0
            state["stall_count"] = 0
            target = clamp_additional_close(target, recovery_target)
            self._log_gripper_guard(
                spec,
                logging.ERROR,
                "HARD LIMIT: backing off to %.4f rad; torque=%s N.m, "
                "current=%s A, overcurrent=%s, hard_fault=%s",
                recovery_target,
                f"{measured_torque:.3f}" if measured_torque is not None else "n/a",
                f"{measured_current:.3f}" if measured_current is not None else "n/a",
                direct_overcurrent,
                hard_status_fault,
            )
            return finish(target)

        if bool(state.get("overcurrent_latched")):
            trip_target = state.get("trip_target")
            if trip_target is None:
                trip_target = pos - closing_dir * max(
                    0.0,
                    float(getattr(self.cfg, "gripper_overcurrent_backoff_rad", 0.04)),
                )
            trip_target = float(trip_target)

            cooldown = max(
                0.0,
                float(getattr(self.cfg, "gripper_overcurrent_cooldown_s", 1.0)),
            )
            cooled = now - float(state.get("trip_time") or 0.0) >= cooldown
            status_guard_enabled = bool(
                getattr(self.cfg, "gripper_status_guard_enabled", True)
            )
            if status_guard_enabled:
                # Do not clear a protection latch merely because feedback has
                # disappeared.  A fresh, healthy status sample is required.
                status_safe = bool(
                    status_ok
                    and status is not None
                    and not status.hard_fault
                    and (
                        measured_torque is None
                        or measured_torque <= release_torque
                    )
                )
            else:
                status_safe = True

            if current_soft_limit is not None or current_hard_limit is not None:
                status_safe = bool(
                    status_safe
                    and current_ok
                    and measured_current is not None
                    and (
                        current_release is None
                        or measured_current <= current_release
                    )
                )
            open_amount = (trip_target - target) * closing_dir
            opened_for_release = open_amount >= max(
                0.0,
                float(self.cfg.gripper_guard_release_rad),
            )
            require_open = bool(
                getattr(self.cfg, "gripper_overcurrent_latch_until_open", True)
            )

            # Opening is always allowed.  Closing remains latched until the
            # cooldown/status conditions are satisfied and, by default, the
            # operator has deliberately opened the hand.
            if opening_requested:
                if cooled and status_safe and (opened_for_release or not require_open):
                    state.update(self._default_gripper_state())
                    self._log_gripper_guard(
                        spec,
                        logging.INFO,
                        "over-current latch cleared by opening command",
                    )
                return finish(target)

            target = clamp_additional_close(target, trip_target)
            self._log_gripper_guard(
                spec,
                logging.WARNING,
                "over-current latch active; blocking close at %.4f rad",
                target,
            )
            return finish(target)

        requested_error = (target - pos) * closing_dir
        release_delta = max(0.0, float(self.cfg.gripper_guard_release_rad))

        # Opening sufficiently from a normal grasp releases pressure control.
        if bool(state.get("grasp_locked")):
            previous_lock_target = state.get("lock_target")
            if previous_lock_target is None:
                previous_lock_target = pos
            previous_lock_target = float(previous_lock_target)
            if (target - previous_lock_target) * closing_dir < -release_delta:
                state["grasp_locked"] = False
                state["lock_target"] = None
                state["stall_count"] = 0
                state["hold_torque_nm"] = 0.0
                state["last_torque_update_time"] = None
                self._log_gripper_guard(
                    spec,
                    logging.INFO,
                    "grasp released by opening command",
                )
                return finish(target)

        # Detect contact from either the live reported torque or the legacy
        # angle-stall detector.  Reaching the soft limit locks immediately so a
        # distant Leader target cannot continue increasing pressure.
        torque_contact = False
        if status_ok and measured_torque is not None and requested_error > 0.0:
            contact_torque = max(
                0.0,
                float(getattr(self.cfg, "gripper_contact_torque_nm", 1.0)),
            )
            barely_moving = movement is not None and movement <= max(
                0.0,
                float(self.cfg.gripper_guard_stall_motion_rad),
            )
            torque_contact = bool(
                (measured_torque >= contact_torque and barely_moving)
                or (soft_limit > 0.0 and measured_torque >= soft_limit)
                or (status is not None and status.stalled)
            )

        last_seen = state.get("last_feedback_ts_seen")
        is_new_sample = sample_ts is not None and sample_ts != last_seen
        stall_contact = False
        if not bool(state.get("grasp_locked")) and is_new_sample:
            state["last_feedback_ts_seen"] = sample_ts
            stall_error = max(0.0, float(self.cfg.gripper_guard_stall_error_rad))
            stall_motion = max(0.0, float(self.cfg.gripper_guard_stall_motion_rad))
            closing_pressure_requested = requested_error > stall_error
            barely_moving = movement is not None and movement <= stall_motion

            if closing_pressure_requested and barely_moving:
                state["stall_count"] = int(state.get("stall_count") or 0) + 1
            else:
                state["stall_count"] = 0

            confirm_count = max(1, int(self.cfg.gripper_guard_stall_confirm_count))
            stall_contact = int(state.get("stall_count") or 0) >= confirm_count

        if not bool(state.get("grasp_locked")) and (torque_contact or stall_contact):
            state["grasp_locked"] = True
            initial_torque = max(
                0.0,
                float(getattr(self.cfg, "gripper_contact_initial_torque_nm", 1.5)),
            )
            if max_torque > 0.0:
                initial_torque = min(initial_torque, max_torque)
            state["hold_torque_nm"] = initial_torque
            state["last_torque_update_time"] = now
            self._log_gripper_guard(
                spec,
                logging.WARNING,
                "contact detected; entering torque-bounded hold "
                "(torque=%s N.m, current=%s A, position=%.4f rad)",
                f"{measured_torque:.3f}" if measured_torque is not None else "n/a",
                f"{measured_current:.3f}" if measured_current is not None else "n/a",
                pos,
            )

        if bool(state.get("grasp_locked")):
            last_update = state.get("last_torque_update_time")
            dt = 0.0 if last_update is None else max(0.0, min(0.1, now - float(last_update)))
            state["last_torque_update_time"] = now

            desired_torque = max_torque
            current_guard_configured = bool(
                current_soft_limit is not None or current_hard_limit is not None
            )
            full_torque_feedback_missing = bool(
                not status_ok or (current_guard_configured and not current_ok)
            )
            if (
                bool(getattr(self.cfg, "gripper_require_status_for_full_torque", True))
                and full_torque_feedback_missing
            ):
                desired_torque = min(
                    desired_torque,
                    max(
                        0.0,
                        float(getattr(self.cfg, "gripper_no_status_max_torque_nm", 1.5)),
                    ),
                )

            hold_torque = max(0.0, float(state.get("hold_torque_nm") or 0.0))
            if hold_torque <= 0.0:
                hold_torque = min(
                    desired_torque,
                    max(
                        0.0,
                        float(getattr(self.cfg, "gripper_contact_initial_torque_nm", 1.5)),
                    ),
                )

            torque_soft_active = bool(
                status_ok
                and measured_torque is not None
                and soft_limit > 0.0
                and measured_torque >= soft_limit
            )
            current_soft_active = bool(
                current_ok
                and measured_current is not None
                and current_soft_limit is not None
                and measured_current >= current_soft_limit
            )
            if torque_soft_active or current_soft_active:
                hold_torque -= max(
                    0.0,
                    float(getattr(self.cfg, "gripper_torque_backoff_nm_s", 6.0)),
                ) * dt
            else:
                torque_below_release = bool(
                    measured_torque is None or measured_torque <= release_torque
                )
                current_below_release = bool(
                    current_release is None
                    or (
                        current_ok
                        and measured_current is not None
                        and measured_current <= current_release
                    )
                )
                feedback_allows_ramp = bool(
                    (status_ok and torque_below_release and current_below_release)
                    or not bool(
                        getattr(self.cfg, "gripper_require_status_for_full_torque", True)
                    )
                )
                if feedback_allows_ramp:
                    hold_torque += max(
                        0.0,
                        float(getattr(self.cfg, "gripper_torque_ramp_nm_s", 2.0)),
                    ) * dt

            # With fresh feedback missing, desired_torque is already capped at
            # the conservative fallback.  Never increase above that cap.
            if full_torque_feedback_missing:
                hold_torque = min(hold_torque, desired_torque)

            hold_torque = max(0.0, min(hold_torque, desired_torque))
            if hard_limit > 0.0:
                hold_torque = min(hold_torque, hard_limit)
            state["hold_torque_nm"] = hold_torque

            hold_error = self._position_error_for_gripper_torque(spec, hold_torque)
            lock_target = pos + closing_dir * hold_error
            state["lock_target"] = lock_target
            target = clamp_additional_close(target, lock_target)

            if status_ok and measured_torque is not None and measured_torque >= soft_limit > 0.0:
                self._log_gripper_guard(
                    spec,
                    logging.WARNING,
                    "soft torque limit active: measured=%.3f N.m, command=%.3f N.m, "
                    "target=%.4f rad",
                    measured_torque,
                    hold_torque,
                    target,
                )
            elif current_soft_active and measured_current is not None:
                self._log_gripper_guard(
                    spec,
                    logging.WARNING,
                    "soft current limit active: measured=%.3f A, command=%.3f N.m, "
                    "target=%.4f rad",
                    measured_current,
                    hold_torque,
                    target,
                )
            return finish(target)

        # Before contact, no software speed/rate limit is applied.  The verified
        # motor-side torque/current limits remain active and provide hard ceilings.
        return finish(target)

    # ------------------------------------------------------------------ #
    # Teleop 値 <-> ラジアン
    # ------------------------------------------------------------------ #
    def _teleop_to_rad(
        self,
        teleop_value: float,
        open_rad: float,
        close_rad: float,
        last_target: Optional[float],
    ) -> float:
        if abs(self._teleop_max - self._teleop_min) < 1e-6:
            norm = 0.0
        else:
            norm = (teleop_value - self._teleop_min) / (self._teleop_max - self._teleop_min)
        norm = max(0.0, min(1.0, norm))
        target = open_rad + norm * (close_rad - open_rad)

        if self.cfg.max_relative_target is not None and last_target is not None:
            max_delta = float(self.cfg.max_relative_target)
            delta = max(-max_delta, min(max_delta, target - last_target))
            target = last_target + delta

        return target

    def _rad_to_teleop(self, pos_rad: float, open_rad: float, close_rad: float) -> float:
        if abs(close_rad - open_rad) < 1e-6:
            norm = 0.0
        else:
            norm = (pos_rad - open_rad) / (close_rad - open_rad)
        norm = max(0.0, min(1.0, norm))
        return float(self._teleop_min + norm * (self._teleop_max - self._teleop_min))

    # ------------------------------------------------------------------ #
    # Robot インターフェース
    # ------------------------------------------------------------------ #
    def connect(self, calibrate: bool = True) -> None:  # type: ignore[override]
        if self._is_connected:
            return

        if calibrate and not self.is_calibrated:
            self.calibrate()

        logger.info(
            "Connecting RSFollower on channel=%s\n%s",
            self.cfg.channel,
            "\n".join(
                f"  {spec.full_name:24s} ID=0x{spec.can_id:02X} {spec.model}"
                for spec in self._motor_specs
            ),
        )

        # Connect/enable の前に現在角度を読み、最初の action は現在角度から補間する。
        # これにより起動直後に目標初期位置へ一気に飛ぶ動きを防ぐ。
        self._prepare_initial_position_ramp()

        for spec in self._motor_specs:
            bus = self._create_bus(spec)
            bus.connect()
            self._bus_by_motor[spec.full_name] = bus

        for cam in self.cameras.values():
            cam.connect()

        self._is_connected = True
        self._start_gripper_guard_feedback()
        self.configure()

        # initial_position が設定されていれば、connect() を返す前に
        # その姿勢への移動を完了させる (推論/teleop はアーム静止後に開始)。
        if self.cfg.initial_position:
            self._move_to_initial_position()

        logger.info("%s RSFollower connected.", self.cfg.id)

    def disconnect(self) -> None:  # type: ignore[override]
        if not self._is_connected:
            return

        # トルクを切る前に初期位置へゆっくり戻る (収録/推論終了後の後片付け)。
        # グリッパガードはランプ中のフィードバックに必要なので停止前に行う。
        self._return_to_initial_position()

        self._stop_gripper_guard_feedback()

        for full_name, bus in list(self._bus_by_motor.items()):
            try:
                bus.disconnect(self.cfg.disable_torque_on_disconnect)
            except Exception:
                logger.exception("Failed to disconnect bus for %s", full_name)

        for name, cam in getattr(self, "cameras", {}).items():
            try:
                cam.disconnect()
            except Exception:
                logger.exception("Failed to disconnect camera %s", name)

        self._bus_by_motor.clear()
        self._is_connected = False
        logger.info("%s RSFollower disconnected.", self.cfg.id)

    def _legacy_feature_key(self, spec: MotorSpec) -> Optional[str]:
        for legacy, new in LEGACY_NAME_ALIASES.items():
            if spec.name == new:
                return f"{spec.side}_{legacy}.pos"
        return None

    def _read_action_value(self, action: Dict[str, Any], spec: MotorSpec) -> float:
        if spec.feature_key in action:
            return float(action[spec.feature_key])
        legacy_key = self._legacy_feature_key(spec)
        if legacy_key is not None and legacy_key in action:
            return float(action[legacy_key])
        return 0.0

    def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:  # type: ignore[override]
        if not self._is_connected:
            raise RuntimeError("RSFollower.send_action() called before connect().")

        missing = [
            spec.full_name for spec in self._motor_specs if spec.full_name not in self._bus_by_motor
        ]
        if missing:
            raise RuntimeError(f"RSFollower bus is not connected for: {missing}")

        # Non-blocking snapshot of each RS05 gripper's type-2 feedback.  This
        # supplies actual torque, status flags and position to the guard before
        # calculating the next command.
        self._refresh_gripper_motor_status()

        targets: Dict[str, float] = {}
        raw_targets: Dict[str, float] = {}
        teleop_values: Dict[str, float] = {}

        for spec in self._motor_specs:
            teleop_value = self._read_action_value(action, spec)
            values = self._ranges[spec.full_name]
            raw_target = self._teleop_to_rad(
                teleop_value,
                values["open"],
                values["close"],
                self._last_targets[spec.full_name],
            )
            target = self._apply_gripper_overcurrent_guard(spec, raw_target, teleop_value)
            teleop_values[spec.full_name] = teleop_value
            raw_targets[spec.full_name] = raw_target
            targets[spec.full_name] = target

        logger.debug(
            "RSFollower CMD teleop: %s -> targets[rad]: %s",
            ", ".join(f"{name}={value:.3f}" for name, value in teleop_values.items()),
            ", ".join(
                f"{name}={targets[name]:.3f}"
                + (f"(raw={raw_targets[name]:.3f})" if abs(targets[name] - raw_targets[name]) > 1e-6 else "")
                for name in targets
            ),
        )

        if self._initial_position_ramp_pending:
            self._run_initial_position_ramp(targets)
        else:
            self._write_goal_positions(targets)

        for full_name, target in targets.items():
            self._last_targets[full_name] = target

        return action

    def get_observation(self) -> Dict[str, Any]:  # type: ignore[override]
        if not self.is_connected:
            raise ConnectionError(f"{self} is not connected.")

        obs: Dict[str, Any] = {}
        for spec in self._motor_specs:
            values = self._ranges[spec.full_name]
            pos_rad = self._last_targets[spec.full_name]
            if pos_rad is None:
                pos_rad = values["open"]
            obs[spec.feature_key] = self._rad_to_teleop(
                pos_rad,
                values["open"],
                values["close"],
            )

        for name, cam in self.cameras.items():
            obs[name] = cam.async_read()

        return obs

    # ------------------------------------------------------------------ #
    # calibrate: 7DOF + Gripper x 左右
    # ------------------------------------------------------------------ #
    def calibrate(self) -> None:  # type: ignore[override]
        logger.info(
            "=== RSFollower calibration using get_angle.py "
            "(7DOF + Gripper, left/right) ==="
        )

        script_path = self._get_angle_script_path()
        if not script_path.is_file():
            raise FileNotFoundError(f"get_angle.py が見つかりません: {script_path}")

        def read_angle_once(motor_id: int, timeout: float = 1.0) -> Optional[float]:
            return self._read_angle_once(motor_id, timeout=timeout)

        def do_calibrate_one_joint(spec: MotorSpec) -> Tuple[float, float]:
            min_val: Optional[float] = None
            max_val: Optional[float] = None
            pos_val: Optional[float] = None

            print("\033[2J\033[H", end="")
            print(
                f"RSFollower Calibration: {spec.full_name} "
                f"(ID=0x{spec.can_id:02X}, {spec.model})"
            )
            print("※ 手でゆっくり動かしてレンジを広げてください。Ctrl+C で確定。\n")

            try:
                while True:
                    angle = read_angle_once(spec.can_id, timeout=1.0)
                    if angle is not None:
                        pos_val = angle
                        if min_val is None or angle < min_val:
                            min_val = angle
                        if max_val is None or angle > max_val:
                            max_val = angle

                    def fmt(v: Optional[float]) -> str:
                        return f"{v:8.4f}" if v is not None else "   ------"

                    def fmt_deg(v: Optional[float]) -> str:
                        return f"{math.degrees(v):6.1f}°" if v is not None else "  ----"

                    print("\033[2J\033[H", end="")
                    print(
                        f"RSFollower Calibration: {spec.full_name} "
                        f"(ID=0x{spec.can_id:02X}, {spec.model}) (Ctrl+C で確定)\n"
                    )
                    print(
                        "NAME                     |    MIN [rad]   (deg) | "
                        "   POS [rad]   (deg) |    MAX [rad]   (deg)"
                    )
                    print(
                        "-------------------------+----------------------+"
                        "----------------------+----------------------"
                    )
                    print(
                        f"{spec.full_name:24s} | "
                        f"{fmt(min_val)} ({fmt_deg(min_val)}) | "
                        f"{fmt(pos_val)} ({fmt_deg(pos_val)}) | "
                        f"{fmt(max_val)} ({fmt_deg(max_val)})"
                    )
                    print("\n※ 関節を全域で数回動かしてください。満足したら Ctrl+C。")
                    time.sleep(0.1)

            except KeyboardInterrupt:
                print(f"\n{spec.full_name} calibration finished.")

            if min_val is None or max_val is None:
                raise RuntimeError(
                    f"{spec.full_name} calibration failed: no angle measured "
                    f"for ID 0x{spec.can_id:02X}"
                )

            if max_val < min_val:
                min_val, max_val = max_val, min_val
            return min_val, max_val

        for spec in self._motor_specs:
            min_rad, max_rad = do_calibrate_one_joint(spec)
            if self._is_inverted(spec):
                self._ranges[spec.full_name] = {"open": max_rad, "close": min_rad}
            else:
                self._ranges[spec.full_name] = {"open": min_rad, "close": max_rad}

        logger.info("Calibration result:\n%s", self._format_ranges_for_log())
        self._save_calibration()
