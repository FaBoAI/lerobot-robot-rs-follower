#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RS05 private protocol を使って、現在角度を 1 回だけ読み出して表示するスクリプト。

- CAN 2.0B / 拡張 29bit / 1 Mbps 前提
- 手順:
    1) Enable (Type 3) を送る
    2) v=0 の Operation Control (Type 1) を何度か送る
    3) その間に返ってくる Type 2 (feedback) を待つ
"""

import argparse
import logging
import math
import sys
import time
from typing import Optional, Tuple

try:
    import can
except ImportError:
    print("python-can がインストールされていません。`pip install python-can` を実行してください。", file=sys.stderr)
    sys.exit(1)

# ---- 定数（rs05_slow_spin.py に合わせる） ----
P_MIN, P_MAX   = -12.57, 12.57   # ≒ -4π..+4π
V_MIN, V_MAX   = -50.0,  50.0
KP_MIN, KP_MAX = 0.0,    500.0
KD_MIN, KD_MAX = 0.0,    5.0
T_MIN, T_MAX   = -5.5,   5.5

MODE_OPCTL = 0x01
MODE_FB    = 0x02
MODE_EN    = 0x03
MODE_STOP  = 0x04


# ---- 汎用ユーティリティ ----

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_u16(x: float, lo: float, hi: float) -> int:
    """[lo, hi] を 0..65535 に線形マッピング。"""
    x = clamp(x, lo, hi)
    span = hi - lo
    if span <= 0:
        return 0
    return int(round((x - lo) * 65535.0 / span))


def u16_to_float(u: int, lo: float, hi: float) -> float:
    """0..65535 を [lo, hi] に線形マッピング。"""
    u = max(0, min(0xFFFF, int(u)))
    span = hi - lo
    if span <= 0:
        return lo
    return lo + span * (u / 65535.0)


def build_ext_id(mode: int, data16: int, id8: int) -> int:
    """
    拡張 ID エンコード:
      bit28..24 : mode (5bit)
      bit23..8  : data16 (16bit)
      bit7..0   : id8 (8bit)
    """
    return ((mode & 0x1F) << 24) | ((data16 & 0xFFFF) << 8) | (id8 & 0xFF)


# ---- RS05 用フレーム生成 ----

def mk_msg_ext(extid: int, payload: bytes = b"\x00" * 8) -> can.Message:
    return can.Message(arbitration_id=extid, is_extended_id=True, data=payload)


def send(bus: "can.BusABC", msg: can.Message, tag: str) -> None:
    try:
        bus.send(msg, timeout=0.2)
        logging.debug("TX %-6s id=0x%08X data=%s", tag, msg.arbitration_id, msg.data.hex(" "))
    except can.CanOperationError:
        logging.warning("送信失敗（ACKなし/送信キュー詰まりの可能性）")


def send_enable(bus: "can.BusABC", motor_id: int) -> None:
    """
    Type 3: enable。data16 は 0 固定（rs05_slow_spin.py と同じ）。
    """
    extid = build_ext_id(MODE_EN, 0x0000, motor_id)
    send(bus, mk_msg_ext(extid), "EN")


def send_opctl_zero(bus: "can.BusABC", motor_id: int,
                    kp: float = 0.0, kd: float = 1.0, t_ff: float = 0.0) -> None:
    """
    v=0, p=0 の Operation Control を 1 発だけ送る。
    → これに対する返信として Type 2 フィードバックが飛んでくる想定。
    """
    data16 = float_to_u16(t_ff, T_MIN, T_MAX)      # Tff を extid.data16 に格納
    extid  = build_ext_id(MODE_OPCTL, data16, motor_id)

    p_u  = float_to_u16(0.0, P_MIN, P_MAX)
    v_u  = float_to_u16(0.0, V_MIN, V_MAX)
    kp_u = float_to_u16(kp,  KP_MIN, KP_MAX)
    kd_u = float_to_u16(kd,  KD_MIN, KD_MAX)

    payload = bytes([
        (p_u >> 8) & 0xFF,  p_u & 0xFF,
        (v_u >> 8) & 0xFF,  v_u & 0xFF,
        (kp_u >> 8) & 0xFF, kp_u & 0xFF,
        (kd_u >> 8) & 0xFF, kd_u & 0xFF,
    ])
    send(bus, mk_msg_ext(extid, payload), "OPCTL")


def decode_angle_from_feedback(msg: can.Message) -> Tuple[float, float]:
    """
    Type 2 フィードバックフレームから角度を [rad], [deg] で返す。

    Byte0-1 : 現在角度（0..65535 → P_MIN..P_MAX）
    それ以外のバイトはここでは使わない。
    """
    b = msg.data
    if len(b) < 2:
        raise ValueError("feedback frame too short")
    pos_u16 = (b[0] << 8) | b[1]
    angle_rad = u16_to_float(pos_u16, P_MIN, P_MAX)
    angle_deg = angle_rad * 180.0 / math.pi
    return angle_rad, angle_deg


def wait_one_feedback(bus: "can.BusABC", motor_id: int,
                      total_timeout: float = 1.0) -> Optional[Tuple[float, float]]:
    """
    enable → OPCTL(v=0) を何回か送りながら、
    Type 2 フィードバックを 1 発だけ拾って角度を返す。
    """
    t_end = time.time() + total_timeout

    # まず enable
    send_enable(bus, motor_id)
    time.sleep(0.02)

    while time.time() < t_end:
        # v=0 の OPCTL を送る
        send_opctl_zero(bus, motor_id)
        # 送ってからちょっとの間だけ受信を覗く
        rx_deadline = time.time() + 0.05
        while time.time() < rx_deadline:
            msg = bus.recv(timeout=0.01)
            if msg is None:
                continue
            if not msg.is_extended_id:
                continue
            mode = (msg.arbitration_id >> 24) & 0x1F
            if mode != MODE_FB:
                continue
            if msg.dlc != 8:
                continue
            # 送信元モータ ID の照合 (2026-08-28 追加): これがないと、バスが混んで
            # いるとき (グリッパ監視 20Hz 等) に他モータのフレームを誤って採用し、
            # 誤った角度 → 座標系誤検出 → 全周回転指令の事故につながる (実害2件)
            responder_id = (msg.arbitration_id >> 8) & 0xFF
            if responder_id != motor_id:
                continue
            logging.debug(
                "RX FB id=0x%08X data=%s",
                msg.arbitration_id, msg.data.hex(" ")
            )
            return decode_angle_from_feedback(msg)

    return None


# ---- メイン ----

def main() -> None:
    ap = argparse.ArgumentParser(description="RS05 から現在角度を 1 回だけ読み出して表示")
    ap.add_argument("--channel", default="can0", help="SocketCAN IF 名（例: can0 / can1）")
    ap.add_argument("--id", required=True, help="モータ ID（例: 0x01 / 1）")
    ap.add_argument("--timeout", type=float, default=1.0, help="タイムアウト [s]")
    ap.add_argument("-v", "--verbose", action="store_true", help="デバッグログを表示")
    args = ap.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s [%(levelname)s] %(message)s")

    try:
        motor_id = int(args.id, 0)
    except ValueError:
        print("モータ ID (--id) の指定が不正です。例: --id 1  または  --id 0x01", file=sys.stderr)
        sys.exit(2)

    if not (0 <= motor_id <= 0x7F):
        print("モータ ID は 0x00〜0x7F の範囲で指定してください。", file=sys.stderr)
        sys.exit(2)

    logging.info("Opening CAN bus: channel=%s (SocketCAN)", args.channel)

    try:
        # DeprecationWarning 対応: bustype ではなく interface を使う
        with can.Bus(interface="socketcan", channel=args.channel) as bus:
            angle = wait_one_feedback(bus, motor_id, total_timeout=args.timeout)
            if angle is None:
                logging.error("タイムアウト %.3f s 内に Type 2 フィードバックが受信できませんでした。", args.timeout)
                sys.exit(1)

            angle_rad, angle_deg = angle
            logging.info("Motor ID 0x%02X 現在角度: %.6f [rad] (%.3f [deg])",
                         motor_id, angle_rad, angle_deg)

    except can.CanError as e:
        logging.error("CAN エラー: %s", e)
        sys.exit(2)


if __name__ == "__main__":
    main()

