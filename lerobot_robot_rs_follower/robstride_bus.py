#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Minimal RobStride private-CAN bus used by the LeRobot follower plugin.

The follower creates one instance per motor.  For ordinary joints this class
keeps the original lightweight position-control behaviour.  For grippers it
can additionally:

* program and verify the motor-side torque limit (``limit_torque``, 0x700B),
* monitor type-2 operation-status frames,
* decode torque, temperature and protection flags,
* decode type-21 fault reports, and
* optionally poll filtered q-axis current (``iqf``, 0x701A).

RobStride private protocol layout:
    ext_id = (communication_type << 24) | (extra_data << 8) | device_id

Operation-control payload is four unsigned big-endian 16-bit values:
    position, velocity, kp, kd
The 16-bit ``extra_data`` field carries feed-forward torque.
"""
from __future__ import annotations

import contextlib
import logging
import math
import struct
import threading
import time
from dataclasses import dataclass, replace
from typing import Any, Dict, Optional, Tuple

import can

logger = logging.getLogger(__name__)

# Private-protocol communication types.
MODE_OPCTL = 0x01
MODE_STATUS = 0x02
MODE_EN = 0x03
MODE_STOP = 0x04
MODE_READ_PARAMETER = 0x11
MODE_WRITE_PARAMETER = 0x12
MODE_FAULT_REPORT = 0x15

# Parameter indexes from the RobStride private protocol.
PARAM_TORQUE_LIMIT = 0x700B
PARAM_CURRENT_LIMIT = 0x7018
PARAM_IQ_FILTERED = 0x701A

# The host ID is placed in the 16-bit extra-data field for host commands.
DEFAULT_HOST_ID = 0xFF

# Command scaling.  RS05 values use the RobStride private-protocol tables.
# RS00/RS03 retain the established command scaling of this plugin to avoid an
# unrelated behaviour change in an existing robot installation.
FOUR_PI = 4.0 * math.pi
MODEL_LIMITS: Dict[str, Dict[str, Tuple[float, float]]] = {
    "RS00": {
        "p": (-12.57, 12.57),
        "v": (-50.0, 50.0),
        "kp": (0.0, 500.0),
        "kd": (0.0, 5.0),
        "t": (-5.5, 5.5),
    },
    "RS03": {
        "p": (-12.57, 12.57),
        "v": (-50.0, 50.0),
        "kp": (0.0, 500.0),
        "kd": (0.0, 5.0),
        "t": (-5.5, 5.5),
    },
    "RS05": {
        # RS05 private-protocol values from the official RS05 manual.
        # Earlier plugin releases incorrectly reused the RS02 +/-17 N.m
        # scale.  That made the decoded RS05 torque about 3.09x too large
        # and caused the gripper guard to release far below the requested
        # physical torque.
        "p": (-FOUR_PI, FOUR_PI),
        "v": (-50.0, 50.0),
        "kp": (0.0, 500.0),
        "kd": (0.0, 5.0),
        "t": (-5.5, 5.5),
    },
}

# Official type-2 feedback scaling tables.  Only RS00/RS03/RS05 are handled by
# this class; RS06 has a separate sender in rs_follower.py.
STATUS_LIMITS: Dict[str, Dict[str, float]] = {
    "RS00": {"p": FOUR_PI, "v": 50.0, "t": 17.0},
    "RS03": {"p": FOUR_PI, "v": 50.0, "t": 60.0},
    "RS05": {"p": FOUR_PI, "v": 50.0, "t": 5.5},
}

# Official RS05 private-protocol limits.  These are validation ceilings for
# user configuration; they are not recommendations to run continuously at the
# peak values.
RS05_MAX_TORQUE_NM = 5.5
RS05_MAX_CURRENT_A = 11.0


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_u16(x: float, lo: float, hi: float) -> int:
    """Map ``[lo, hi]`` linearly to ``0..65535``."""
    x = clamp(float(x), float(lo), float(hi))
    span = float(hi) - float(lo)
    if span <= 0.0:
        return 0
    return int(round((x - float(lo)) * 65535.0 / span))


def u16_to_symmetric(value: int, magnitude: float) -> float:
    """Decode the RobStride unsigned 16-bit symmetric representation."""
    return (float(value) / 0x7FFF - 1.0) * float(magnitude)


def build_ext_id(mode: int, data16: int, id8: int) -> int:
    return ((int(mode) & 0x1F) << 24) | ((int(data16) & 0xFFFF) << 8) | (int(id8) & 0xFF)


def mk_msg_ext(extid: int, payload: bytes = b"\x00" * 8) -> can.Message:
    return can.Message(
        arbitration_id=int(extid),
        is_extended_id=True,
        data=bytes(payload),
    )


@dataclass
class MinimalMotor:
    id: int
    name: str = "qdd0"


@dataclass(frozen=True)
class MotorStatus:
    """Latest RobStride type-2/fault status for one motor."""

    timestamp: float
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    uncalibrated: bool = False
    stalled: bool = False
    magnetic_encoder_fault: bool = False
    overtemperature: bool = False
    overcurrent: bool = False
    undervoltage: bool = False
    fault_report: bool = False
    fault_stall_current: bool = False
    fault_encoder_uncalibrated: bool = False
    fault_overvoltage: bool = False
    fault_undervoltage: bool = False
    fault_gate: bool = False
    fault_motor_overtemperature: bool = False
    warning_motor_overtemperature: bool = False
    current_a: Optional[float] = None
    current_timestamp: Optional[float] = None

    @property
    def hard_fault(self) -> bool:
        """True for conditions that require immediate gripper backoff."""
        return bool(
            self.overcurrent
            or self.overtemperature
            or self.magnetic_encoder_fault
            # Receiving a type-21 report is not itself a fault.  Some
            # firmware versions can return a zero-valued report; only decoded
            # fault bits should trigger the hard-fault path.
            or self.fault_stall_current
            or self.fault_encoder_uncalibrated
            or self.fault_overvoltage
            or self.fault_undervoltage
            or self.fault_gate
            or self.fault_motor_overtemperature
        )


class RobStrideBus:
    """Small compatibility layer for one RobStride motor.

    Parameters used only by the gripper safety path are optional, so existing
    non-gripper joints keep the previous low-overhead behaviour.
    """

    def __init__(
        self,
        channel: str = "can0",
        motor_id: int = 0x01,
        kp: float = 20.0,
        kd: float = 1.0,
        default_v_set: float = 0.0,
        t_ff: float = 0.0,
        tx_hz: float = 50.0,
        model: str = "RS05",
        *,
        monitor_feedback: bool = False,
        feedback_stale_s: float = 0.25,
        hardware_torque_limit_nm: Optional[float] = None,
        hardware_torque_limit_required: bool = False,
        hardware_torque_limit_verify: bool = True,
        hardware_current_limit_a: Optional[float] = None,
        hardware_current_limit_required: bool = False,
        hardware_current_limit_verify: bool = True,
        current_monitor_hz: float = 0.0,
        host_id: int = DEFAULT_HOST_ID,
    ) -> None:
        self.channel = str(channel)
        self.model = str(model).upper()
        limits = MODEL_LIMITS.get(self.model, MODEL_LIMITS["RS05"])
        self.p_min, self.p_max = limits["p"]
        self.v_min, self.v_max = limits["v"]
        self.kp_min, self.kp_max = limits["kp"]
        self.kd_min, self.kd_max = limits["kd"]
        self.t_min, self.t_max = limits["t"]

        self._bus: Optional[Any] = None
        self.is_connected: bool = False
        self.motors: Dict[str, MinimalMotor] = {"qdd0": MinimalMotor(id=int(motor_id))}
        self.kp = float(kp)
        self.kd = float(kd)
        self.default_v_set = float(default_v_set)
        self.t_ff = float(t_ff)
        self.period = 1.0 / max(float(tx_hz), 1.0)
        self._last_goal_pos: Dict[str, float] = {"qdd0": 0.0}

        self.monitor_feedback = bool(monitor_feedback)
        self.feedback_stale_s = max(0.0, float(feedback_stale_s))
        self.hardware_torque_limit_nm = (
            None if hardware_torque_limit_nm is None else max(0.0, float(hardware_torque_limit_nm))
        )
        if (
            self.model == "RS05"
            and self.hardware_torque_limit_nm is not None
            and self.hardware_torque_limit_nm > RS05_MAX_TORQUE_NM
        ):
            raise ValueError(
                f"RS05 torque limit {self.hardware_torque_limit_nm:.3f} N.m exceeds "
                f"the protocol/peak ceiling {RS05_MAX_TORQUE_NM:.3f} N.m"
            )
        self.hardware_torque_limit_required = bool(hardware_torque_limit_required)
        self.hardware_torque_limit_verify = bool(hardware_torque_limit_verify)
        self.hardware_current_limit_a = (
            None if hardware_current_limit_a is None else max(0.0, float(hardware_current_limit_a))
        )
        if (
            self.model == "RS05"
            and self.hardware_current_limit_a is not None
            and self.hardware_current_limit_a > RS05_MAX_CURRENT_A
        ):
            raise ValueError(
                f"RS05 current limit {self.hardware_current_limit_a:.3f} A exceeds "
                f"the protocol ceiling {RS05_MAX_CURRENT_A:.3f} A"
            )
        self.hardware_current_limit_required = bool(hardware_current_limit_required)
        self.hardware_current_limit_verify = bool(hardware_current_limit_verify)
        self.current_monitor_hz = max(0.0, float(current_monitor_hz))
        self.host_id = int(host_id) & 0xFF

        self._status_lock = threading.Lock()
        self._latest_status: Optional[MotorStatus] = None
        self._parameter_values: Dict[int, Tuple[float, float]] = {}
        self._rx_stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._last_current_request_time = 0.0

    @property
    def motor_id(self) -> int:
        return int(self.motors["qdd0"].id)

    # ------------------------------------------------------------------
    # Low-level transmit / receive
    # ------------------------------------------------------------------
    def _send(self, msg: can.Message, tag: str) -> bool:
        if self._bus is None:
            raise RuntimeError("RobStride CAN bus is not open")
        try:
            self._bus.send(msg, timeout=0.2)
            logger.debug(
                "TX %-8s id=0x%08X data=%s",
                tag,
                int(msg.arbitration_id),
                bytes(msg.data).hex(" "),
            )
            return True
        except Exception as exc:
            logger.warning(
                "RobStride send failed (%s, id=0x%02X): %s",
                tag,
                self.motor_id,
                exc,
            )
            return False

    def _enable(self, motor_id: int) -> bool:
        extid = build_ext_id(MODE_EN, self.host_id, motor_id)
        return self._send(mk_msg_ext(extid), "ENABLE")

    def _stop(self, motor_id: int) -> bool:
        extid = build_ext_id(MODE_STOP, self.host_id, motor_id)
        return self._send(mk_msg_ext(extid), "STOP")

    def try_fault_recovery(self) -> bool:
        """モータ内部保護 (堵転/過電流等) でフォルトした後の復旧を試みる。

        RobStride 私有プロトコルでは Type4 (STOP) の data[0]=1 が故障クリア。
        クリア → 再イネーブル → トルク/電流上限の再書き込みの順に送る。
        ⚠️ limit_torque/limit_cur はフォルト内部リセットで工場既定値
        (5.5 N.m / 11 A) に戻る (2026-08-29 実測: フォルト歴のある 0x01 が
        既定値に戻り、無フォルトの 0x11 は設定値を保持していた)。
        復旧のたびに必ず書き直す。
        """
        extid = build_ext_id(MODE_STOP, self.host_id, self.motor_id)
        ok_clear = self._send(
            mk_msg_ext(extid, b"\x01" + b"\x00" * 7), "STOP+FAULT_CLEAR"
        )
        time.sleep(0.02)
        ok_enable = self._enable(self.motor_id)
        if self.hardware_torque_limit_nm is not None and self.hardware_torque_limit_nm > 0.0:
            self._write_float_parameter(PARAM_TORQUE_LIMIT, self.hardware_torque_limit_nm)
        if self.hardware_current_limit_a is not None and self.hardware_current_limit_a > 0.0:
            self._write_float_parameter(PARAM_CURRENT_LIMIT, self.hardware_current_limit_a)
        return bool(ok_clear and ok_enable)

    def _op_control(
        self,
        motor_id: int,
        p_set: float,
        v_set: float,
        kp: float,
        kd: float,
        t_ff: float,
    ) -> None:
        data16 = float_to_u16(t_ff, self.t_min, self.t_max)
        extid = build_ext_id(MODE_OPCTL, data16, motor_id)

        p_u = float_to_u16(p_set, self.p_min, self.p_max)
        v_u = float_to_u16(v_set, self.v_min, self.v_max)
        kp_u = float_to_u16(kp, self.kp_min, self.kp_max)
        kd_u = float_to_u16(kd, self.kd_min, self.kd_max)

        payload = struct.pack(">HHHH", p_u, v_u, kp_u, kd_u)
        self._send(mk_msg_ext(extid, payload), "OPCTL")

    def _write_float_parameter(self, parameter_id: int, value: float) -> bool:
        payload = struct.pack("<HHf", int(parameter_id) & 0xFFFF, 0, float(value))
        extid = build_ext_id(MODE_WRITE_PARAMETER, self.host_id, self.motor_id)
        return self._send(mk_msg_ext(extid, payload), f"WR-{parameter_id:04X}")

    def _request_float_parameter(self, parameter_id: int) -> bool:
        payload = struct.pack("<HHL", int(parameter_id) & 0xFFFF, 0, 0)
        extid = build_ext_id(MODE_READ_PARAMETER, self.host_id, self.motor_id)
        return self._send(mk_msg_ext(extid, payload), f"RD-{parameter_id:04X}")

    def _decode_status_frame(self, frame: can.Message) -> Optional[Dict[str, Any]]:
        if not bool(frame.is_extended_id):
            return None

        arbitration_id = int(frame.arbitration_id)
        communication_type = (arbitration_id >> 24) & 0x1F
        extra_data = (arbitration_id >> 8) & 0xFFFF
        responder_id = extra_data & 0xFF
        host_id = arbitration_id & 0xFF

        if responder_id != self.motor_id:
            return None

        data = bytes(frame.data)
        now = time.monotonic()

        if communication_type == MODE_STATUS:
            if len(data) < 8:
                return None
            position_u16, velocity_u16, torque_u16, temperature_u16 = struct.unpack(">HHHH", data[:8])
            limits = STATUS_LIMITS.get(self.model, STATUS_LIMITS["RS05"])
            with self._status_lock:
                previous = self._latest_status
                status = MotorStatus(
                    timestamp=now,
                    position=u16_to_symmetric(position_u16, limits["p"]),
                    velocity=u16_to_symmetric(velocity_u16, limits["v"]),
                    torque=u16_to_symmetric(torque_u16, limits["t"]),
                    temperature=float(temperature_u16) * 0.1,
                    uncalibrated=bool((extra_data >> 13) & 0x01),
                    stalled=bool((extra_data >> 12) & 0x01),
                    magnetic_encoder_fault=bool((extra_data >> 11) & 0x01),
                    overtemperature=bool((extra_data >> 10) & 0x01),
                    overcurrent=bool((extra_data >> 9) & 0x01),
                    undervoltage=bool((extra_data >> 8) & 0x01),
                    current_a=(previous.current_a if previous is not None else None),
                    current_timestamp=(
                        previous.current_timestamp if previous is not None else None
                    ),
                )
                self._latest_status = status
            return {
                "timestamp": now,
                "communication_type": communication_type,
                "motor_id": responder_id,
                "host_id": host_id,
                "status": status,
            }

        if communication_type == MODE_FAULT_REPORT:
            if len(data) < 8:
                return None
            fault_value, warning_value = struct.unpack("<LL", data[:8])
            with self._status_lock:
                previous = self._latest_status
                status = MotorStatus(
                    timestamp=now,
                    position=(previous.position if previous is not None else 0.0),
                    velocity=(previous.velocity if previous is not None else 0.0),
                    torque=(previous.torque if previous is not None else 0.0),
                    temperature=(previous.temperature if previous is not None else 0.0),
                    uncalibrated=(previous.uncalibrated if previous is not None else False),
                    stalled=(previous.stalled if previous is not None else False),
                    magnetic_encoder_fault=(
                        previous.magnetic_encoder_fault if previous is not None else False
                    ),
                    overtemperature=(
                        previous.overtemperature if previous is not None else False
                    ),
                    overcurrent=(previous.overcurrent if previous is not None else False),
                    undervoltage=(previous.undervoltage if previous is not None else False),
                    fault_report=True,
                    warning_motor_overtemperature=bool((warning_value >> 0) & 0x01),
                    fault_stall_current=bool((fault_value >> 14) & 0x01),
                    fault_encoder_uncalibrated=bool((fault_value >> 7) & 0x01),
                    fault_overvoltage=bool((fault_value >> 3) & 0x01),
                    fault_undervoltage=bool((fault_value >> 2) & 0x01),
                    fault_gate=bool((fault_value >> 1) & 0x01),
                    fault_motor_overtemperature=bool((fault_value >> 0) & 0x01),
                    current_a=(previous.current_a if previous is not None else None),
                    current_timestamp=(
                        previous.current_timestamp if previous is not None else None
                    ),
                )
                self._latest_status = status
            return {
                "timestamp": now,
                "communication_type": communication_type,
                "motor_id": responder_id,
                "host_id": host_id,
                "status": status,
            }

        if communication_type == MODE_READ_PARAMETER and len(data) >= 8:
            parameter_id = struct.unpack_from("<H", data, 0)[0]
            value = struct.unpack_from("<f", data, 4)[0]
            with self._status_lock:
                self._parameter_values[int(parameter_id)] = (float(value), now)
                if int(parameter_id) == PARAM_IQ_FILTERED:
                    if self._latest_status is None:
                        self._latest_status = MotorStatus(
                            timestamp=now,
                            current_a=float(value),
                            current_timestamp=now,
                        )
                    else:
                        self._latest_status = replace(
                            self._latest_status,
                            current_a=float(value),
                            current_timestamp=now,
                        )
            return {
                "timestamp": now,
                "communication_type": communication_type,
                "motor_id": responder_id,
                "host_id": host_id,
                "parameter_id": int(parameter_id),
                "parameter_value": float(value),
            }

        return None

    def _receive_and_decode(self, timeout: float) -> Optional[Dict[str, Any]]:
        if self._bus is None:
            return None
        try:
            frame = self._bus.recv(timeout=max(0.0, float(timeout)))
        except Exception as exc:
            if not self._rx_stop.is_set():
                logger.debug("RobStride recv failed id=0x%02X: %s", self.motor_id, exc)
            return None
        if frame is None:
            return None
        return self._decode_status_frame(frame)

    def _wait_for_parameter_value(
        self,
        parameter_id: int,
        *,
        sent_after: float,
        timeout: float,
    ) -> Optional[float]:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            decoded = self._receive_and_decode(min(0.03, remaining))
            if not decoded:
                continue
            if (
                decoded.get("communication_type") == MODE_READ_PARAMETER
                and int(decoded.get("parameter_id", -1)) == int(parameter_id)
                and float(decoded.get("timestamp", 0.0)) >= float(sent_after)
            ):
                return float(decoded["parameter_value"])
        return None

    def _wait_for_status(self, timeout: float) -> Optional[Dict[str, Any]]:
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            decoded = self._receive_and_decode(min(0.03, remaining))
            if decoded and decoded.get("communication_type") in (MODE_STATUS, MODE_FAULT_REPORT):
                return decoded
        return None

    def _write_and_verify_float_parameter(
        self,
        parameter_id: int,
        value: float,
        *,
        verify: bool,
        retries: int = 2,
        tolerance: float = 0.05,
    ) -> bool:
        for attempt in range(1, max(1, int(retries)) + 1):
            if not self._write_float_parameter(parameter_id, value):
                continue

            # A parameter write normally produces a type-2 status frame.  Drain
            # it before issuing the read-back request.
            self._wait_for_status(timeout=0.12)

            if not verify:
                return True

            sent_after = time.monotonic()
            if not self._request_float_parameter(parameter_id):
                continue
            read_back = self._wait_for_parameter_value(
                parameter_id,
                sent_after=sent_after,
                timeout=0.18,
            )
            if read_back is not None and abs(float(read_back) - float(value)) <= float(tolerance):
                logger.info(
                    "RobStride id=0x%02X verified parameter 0x%04X = %.3f",
                    self.motor_id,
                    parameter_id,
                    read_back,
                )
                return True

            logger.warning(
                "RobStride id=0x%02X parameter 0x%04X verify attempt %d failed "
                "(requested=%.3f, read_back=%s)",
                self.motor_id,
                parameter_id,
                attempt,
                value,
                "none" if read_back is None else f"{read_back:.3f}",
            )
        return False

    # ------------------------------------------------------------------
    # Background status monitor
    # ------------------------------------------------------------------
    def _start_rx_thread(self) -> None:
        if not self.monitor_feedback:
            return
        if self._rx_thread is not None and self._rx_thread.is_alive():
            return
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop,
            name=f"RobStrideRX-{self.motor_id:02X}",
            daemon=True,
        )
        self._rx_thread.start()

    def _stop_rx_thread(self) -> None:
        self._rx_stop.set()
        thread = self._rx_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)
        self._rx_thread = None

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            self._receive_and_decode(timeout=0.05)

    # ------------------------------------------------------------------
    # Public API used by RSFollower
    # ------------------------------------------------------------------
    def connect(self) -> None:
        if self.is_connected:
            return

        try:
            try:
                self._bus = can.Bus(interface="socketcan", channel=self.channel)
            except TypeError:
                # Compatibility with older python-can releases.
                self._bus = can.Bus(bustype="socketcan", channel=self.channel)

            wants_limits = (
                self.hardware_torque_limit_nm is not None
                or self.hardware_current_limit_a is not None
            )
            if self.monitor_feedback or wants_limits:
                # If a previous process crashed while the motor was enabled,
                # stop it before changing safety limits.
                # (monitor_feedback なしの関節でも、トルク上限指定時はここを通す)
                self._stop(self.motor_id)
                self._wait_for_status(timeout=0.10)

                if self.hardware_torque_limit_nm is not None:
                    ok = self._write_and_verify_float_parameter(
                        PARAM_TORQUE_LIMIT,
                        self.hardware_torque_limit_nm,
                        verify=self.hardware_torque_limit_verify,
                    )
                    if not ok:
                        message = (
                            f"RobStride gripper id=0x{self.motor_id:02X}: failed to set/verify "
                            f"torque limit {self.hardware_torque_limit_nm:.3f} N.m"
                        )
                        if self.hardware_torque_limit_required:
                            raise RuntimeError(message)
                        logger.error("%s; continuing because required=false", message)

                if self.hardware_current_limit_a is not None:
                    ok = self._write_and_verify_float_parameter(
                        PARAM_CURRENT_LIMIT,
                        self.hardware_current_limit_a,
                        verify=self.hardware_current_limit_verify,
                    )
                    if not ok:
                        message = (
                            f"RobStride gripper id=0x{self.motor_id:02X}: failed to set/verify "
                            f"current limit {self.hardware_current_limit_a:.3f} A"
                        )
                        if self.hardware_current_limit_required:
                            raise RuntimeError(message)
                        logger.error("%s; continuing because required=false", message)

            if not self._enable(self.motor_id):
                raise RuntimeError(f"Failed to send enable to RobStride id=0x{self.motor_id:02X}")

            if self.monitor_feedback:
                status = self._wait_for_status(timeout=0.18)
                if status is None:
                    logger.warning(
                        "RobStride gripper id=0x%02X: no type-2 status after enable",
                        self.motor_id,
                    )

            self.is_connected = True
            self._start_rx_thread()
            logger.info(
                "RobStrideBus(%s, id=0x%02X) connected on %s%s",
                self.model,
                self.motor_id,
                self.channel,
                (
                    (f"; torque_limit={self.hardware_torque_limit_nm:.3f} N.m"
                     if self.hardware_torque_limit_nm is not None else "")
                    +
                    (f"; current_limit={self.hardware_current_limit_a:.3f} A"
                     if self.hardware_current_limit_a is not None else "")
                ),
            )
        except Exception:
            self._stop_rx_thread()
            if self._bus is not None:
                try:
                    self._stop(self.motor_id)
                except Exception:
                    pass
                try:
                    self._bus.shutdown()
                except Exception:
                    pass
            self._bus = None
            self.is_connected = False
            raise

    def disconnect(self, disable_torque_on_disconnect: bool = True) -> None:
        if self._bus is None:
            self.is_connected = False
            return

        if disable_torque_on_disconnect:
            try:
                self._stop(self.motor_id)
                time.sleep(0.02)
            except Exception:
                logger.debug("Failed to stop id=0x%02X during disconnect", self.motor_id, exc_info=True)

        self._stop_rx_thread()
        try:
            self._bus.shutdown()
        finally:
            self._bus = None
            self.is_connected = False
        logger.info("RobStrideBus(%s, id=0x%02X) disconnected", self.model, self.motor_id)

    @contextlib.contextmanager
    def torque_disabled(self):
        """Compatibility context manager used by older follower code."""
        yield

    def configure_motors(self) -> None:
        return

    def sync_write(self, register: str, goal_pos: Dict[str, float]) -> None:
        if not self.is_connected:
            raise RuntimeError("RobStrideBus not connected")
        if register != "Goal_Position":
            raise NotImplementedError(
                f"sync_write only supports 'Goal_Position', got {register}"
            )

        p_set = float(goal_pos.get("qdd0", 0.0))
        self._last_goal_pos["qdd0"] = p_set
        self._op_control(
            self.motor_id,
            p_set=p_set,
            v_set=self.default_v_set,
            kp=self.kp,
            kd=self.kd,
            t_ff=self.t_ff,
        )

        if self.monitor_feedback and self.current_monitor_hz > 0.0:
            now = time.monotonic()
            current_period = 1.0 / self.current_monitor_hz
            if now - self._last_current_request_time >= current_period:
                self._last_current_request_time = now
                self._request_float_parameter(PARAM_IQ_FILTERED)

    def set_velocity(self, vel_rad_s: float) -> None:
        if not self.is_connected:
            raise RuntimeError("RobStrideBus not connected")
        self._op_control(
            self.motor_id,
            p_set=self._last_goal_pos["qdd0"],
            v_set=float(vel_rad_s),
            kp=self.kp,
            kd=self.kd,
            t_ff=self.t_ff,
        )

    def get_latest_status(self) -> Optional[MotorStatus]:
        """Return the most recent immutable motor-status snapshot."""
        with self._status_lock:
            return self._latest_status

    def set_hardware_torque_limit(self, torque_nm: float, verify: bool = True) -> bool:
        """Update the motor-side torque ceiling while connected."""
        if self._bus is None:
            raise RuntimeError("RobStrideBus not connected")
        # The background receiver would race with a synchronous read-back.  At
        # runtime we therefore issue the write and rely on subsequent type-2
        # status monitoring; connect() performs the full verified read-back.
        self.hardware_torque_limit_nm = max(0.0, float(torque_nm))
        if self._rx_thread is not None and self._rx_thread.is_alive():
            if verify:
                logger.warning(
                    "Runtime torque-limit update cannot be synchronously read back while RX monitor is active"
                )
            return self._write_float_parameter(PARAM_TORQUE_LIMIT, self.hardware_torque_limit_nm)
        return self._write_and_verify_float_parameter(
            PARAM_TORQUE_LIMIT,
            self.hardware_torque_limit_nm,
            verify=bool(verify),
        )
