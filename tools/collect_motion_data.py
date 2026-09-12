#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人体运动数据采集器 —— 零固件改动版
=====================================

依据《数据采集方案_零阻力台架穿戴_v1.md》§5.1 的"最小第一步":
板侧 1 kHz 上报帧已含全部运动量, 本工具只做 PC 侧接收 + 落盘。

数据来源 : F407 经以太网 UDP 5001 每 1 ms 一帧 (TIM6 ISR 内 report_frame_build)
帧格式   : ReportFrame_t = FrameHeader(8B) + JointStatus[6](17B×6) + crc16(2B) = 112B
字段单位 : position mdeg | velocity mdeg/s | torque mNm | temperature 0.1°C
板上时钟 : header.timestamp = HAL_GetTick() (ms) —— 以此为准, PC 到达时刻另存供漂移监测

用法示例:
    python collect_motion_data.py --label G0_baseline --duration 30
    python collect_motion_data.py --label G1_LHip_1.0Hz --subject S1 --load 0
    python collect_motion_data.py --label G4_shoulder_2kg --load 2 --load-pos "右手, r=0.35m"

产物 (outdir 默认 ./data):
    session_YYYYMMDD_HHMM_<label>.bin  原始 112B 帧拼接 (无损, 主记录)
    session_YYYYMMDD_HHMM_<label>.arr  uint64 每帧 PC 到达时刻(ns), 8B/帧
    session_YYYYMMDD_HHMM_<label>.json 元数据 + 统计 + 字段定义
    session_YYYYMMDD_HHMM_<label>.csv  (可选 --csv) 解码后 SI 单位

质控(方案 §10): 采样率/抖动/seq 缺口/CRC 错误/帧长错误/tau 饱和 —— 结束时打印并写入 json。
"""

import argparse
import json
import os
import socket
import struct
import sys
import time
from datetime import datetime, timezone, timedelta

# ----------------------------------------------------------------------------
# 1. 协议常量 (严格对照 App/bsp/contract.h 与 App/udp_proto/udp_protocol.c)
# ----------------------------------------------------------------------------
UDP_PORT_DEFAULT = 5001

FRAME_HEADER_FMT = '<BBHI'          # frame_type, reserved, seq_num, timestamp
FRAME_HEADER_SIZE = 8
JOINT_STATUS_FMT = '<BiiiHH'        # joint_id, pos(mdeg), vel(mdeg/s), tau(mNm), temp(0.1C), fault
JOINT_STATUS_SIZE = 17
REPORT_FRAME_FMT = '<' + 'BBHI' + 'BiiiHH' * 6 + 'H'
REPORT_FRAME_SIZE = FRAME_HEADER_SIZE + 6 * JOINT_STATUS_SIZE + 2      # 112

FRAME_TYPE_REPORT = 0x01

# 关节顺序 = joint_unit.c 注册表顺序 (index 0..5)
JOINT_ORDER = ['L-Hip', 'R-Hip', 'L-Shldr', 'L-Elbow', 'R-Shldr', 'R-Elbow']
JOINT_BUS = ['CAN1', 'CAN1', 'CAN2', 'CAN2', 'CAN2', 'CAN2']
JOINT_MOTOR = ['CyberGear', 'CyberGear', 'RS01', 'RS01', 'RS01', 'RS01']

assert struct.calcsize(REPORT_FRAME_FMT) == REPORT_FRAME_SIZE, "ReportFrame 尺寸不匹配"

# ----------------------------------------------------------------------------
# 2. CRC16 (MODBUS, init=0xFFFF, poly=0xA001) —— 表驱动, 与 C 实现逐位等价
# ----------------------------------------------------------------------------
def _make_crc_table():
    tbl = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
        tbl.append(crc)
    return tbl

_CRC_TABLE = _make_crc_table()


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def crc16_naive(data: bytes) -> int:
    """逐位版, 仅用于 --selftest 校验表驱动实现"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 0x0001) else (crc >> 1)
    return crc


# ----------------------------------------------------------------------------
# 3. 采集器
# ----------------------------------------------------------------------------
class Collector:
    def __init__(self, args):
        self.args = args
        self.host = args.host
        self.port = args.port
        self.outdir = args.outdir
        os.makedirs(self.outdir, exist_ok=True)

        stamp = datetime.now().strftime('%Y%m%d_%H%M')
        safe_label = ''.join(c if (c.isalnum() or c in '-_.') else '_' for c in args.label)
        base = os.path.join(self.outdir, f'session_{stamp}_{safe_label}')
        self.f_bin = base + '.bin'
        self.f_arr = base + '.arr'
        self.f_json = base + '.json'
        self.f_csv = base + '.csv' if args.csv else None

        # 统计
        self.n_recv = 0            # 收到的 UDP 数据报
        self.n_valid = 0           # 通过长度+CRC 的帧
        self.err_size = 0
        self.err_crc = 0
        self.err_type = 0
        self.gaps = []             # (prev_seq, seq, missing)
        self.n_missing = 0
        self.last_seq = None
        self.seq_wrap = 0
        self.ts_first = None
        self.ts_last = None
        self.ts_prev = None
        self.dt_us_list = []       # 板上时间戳间隔(us), 用于抖动
        self.dt_big = []           # 超阈值间隔 (board_ts_ms, dt_ms) —— 抓 ISR 阻塞造成的"时间空洞"
        self.dt_big_total_ms = 0.0
        self.dt_max_ms = 0.0
        self.arr_first = None
        self.arr_last = None
        self.jpos_min = [None] * 6
        self.jpos_max = [None] * 6
        self.tau_min = [None] * 6
        self.tau_max = [None] * 6
        self.tau_sat = [0] * 6     # |tau| >= 17000 mNm 视为饱和(RS01 峰值 17Nm)
        self.temperature = [None] * 6
        self.fault_seen = set()

    # ---- 单帧处理 ----
    def handle(self, pkt: bytes, arr_ns: int):
        self.n_recv += 1
        if len(pkt) != REPORT_FRAME_SIZE:
            self.err_size += 1
            return None
        if crc16(pkt[:-2]) != struct.unpack_from('<H', pkt, REPORT_FRAME_SIZE - 2)[0]:
            self.err_crc += 1
            return None
        vals = struct.unpack(REPORT_FRAME_FMT, pkt)
        ftype, _rsv, seq, ts = vals[0], vals[1], vals[2], vals[3]
        if ftype != FRAME_TYPE_REPORT:
            self.err_type += 1
            return None

        # seq 缺口 (uint16 回绕)
        if self.last_seq is not None:
            d = (seq - self.last_seq) & 0xFFFF
            if d == 0:
                pass
            elif d > 1:
                self.gaps.append((self.last_seq, seq, d - 1))
                self.n_missing += d - 1
                if seq < self.last_seq:
                    self.seq_wrap += 1
        self.last_seq = seq

        # 板上时间戳: 间隔统计 (>0 且合理才计)
        if self.ts_first is None:
            self.ts_first = ts
            self.arr_first = arr_ns
        else:
            dms = (ts - self.ts_prev) & 0xFFFFFFFF
            if 0 < dms < 100:
                self.dt_us_list.append(dms * 1000)
            if 1 < dms < 60000:                       # 有效间隔: 统计最大值与超阈值事件
                if dms > self.dt_max_ms:
                    self.dt_max_ms = float(dms)
                if dms > self.args.gap_ms:
                    self.dt_big.append((ts, dms))
                    self.dt_big_total_ms += dms - 1.0
        self.ts_prev = ts
        self.ts_last = ts
        self.arr_last = arr_ns

        # 每关节物理量
        for j in range(6):
            off = FRAME_HEADER_SIZE + j * JOINT_STATUS_SIZE
            jid, pos, vel, tau, temp, fault = struct.unpack_from(JOINT_STATUS_FMT, pkt, off)
            p = pos / 1000.0            # deg
            t = tau / 1000.0            # N·m
            if self.jpos_min[j] is None or p < self.jpos_min[j]:
                self.jpos_min[j] = p
            if self.jpos_max[j] is None or p > self.jpos_max[j]:
                self.jpos_max[j] = p
            if self.tau_min[j] is None or t < self.tau_min[j]:
                self.tau_min[j] = t
            if self.tau_max[j] is None or t > self.tau_max[j]:
                self.tau_max[j] = t
            if abs(tau) >= 17000:
                self.tau_sat[j] += 1
            self.temperature[j] = temp / 10.0
            if fault:
                self.fault_seen.add((j, fault))
        self.n_valid += 1
        return vals

    # ---- 统计输出 ----
    def stats(self):
        dur_s = 0.0
        if self.arr_first and self.arr_last:
            dur_s = (self.arr_last - self.arr_first) / 1e9
        rate = (self.n_valid / dur_s) if dur_s > 0 else 0.0
        dt = sorted(self.dt_us_list)
        jit = 0.0
        if len(dt) > 16:
            n = len(dt)
            p50 = dt[n // 2]
            jit = (dt[int(n * 0.99)] - p50) / 1000.0     # p50→p99 展宽(ms)
        board_dur = 0.0
        if self.ts_first is not None and self.ts_last is not None:
            board_dur = ((self.ts_last - self.ts_first) & 0xFFFFFFFF) / 1000.0
        return {
            'udp_datagrams': self.n_recv,
            'valid_frames': self.n_valid,
            'duration_pc_s': round(dur_s, 3),
            'duration_board_s': round(board_dur, 3),
            'rate_hz': round(rate, 2),
            'expected_rate_hz': 1000,
            'err_size': self.err_size,
            'err_crc': self.err_crc,
            'err_type': self.err_type,
            'seq_missing_frames': self.n_missing,
            'loss_rate_pct': round(100.0 * self.n_missing / max(1, self.n_valid + self.n_missing), 4),
            'seq_gap_events': len(self.gaps),
            'seq_wrap_count': self.seq_wrap,
            'gap_events_first20': self.gaps[:20],
            'board_max_dt_ms': round(self.dt_max_ms, 3),
            'board_time_gap_events': len(self.dt_big),
            'board_time_gap_total_ms': round(self.dt_big_total_ms, 1),
            'coverage_pct': round(100.0 * self.n_valid / max(1.0, board_dur * 1000.0 + 1.0), 3),
            'jitter_p50_to_p99_ms': round(jit, 4),
        }

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rcvbuf = self.args.rcvbuf
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        except OSError:
            pass
        actual = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        s.bind((self.host, self.port))
        s.settimeout(1.0)

        print(f"[MOSS] 已监听 UDP {self.host}:{self.port}  (SO_RCVBUF={actual} B)")
        print(f"[MOSS] 标签={self.args.label}  时长={self.args.duration or '手动(Ctrl-C)'} s")
        print(f"[MOSS] 落盘: {self.f_bin}")
        print(f"[MOSS] 确认下位机处于 ZERO(零阻力)模式、其余模式已禁用, 再让受试者开始动作。\n")

        fbin = open(self.f_bin, 'wb', buffering=1024 * 256)
        farr = open(self.f_arr, 'wb', buffering=1024 * 256)
        fcsv = open(self.f_csv, 'w', buffering=1024 * 256) if self.f_csv else None
        if fcsv:
            hdr = ['arrival_ns', 'board_ts_ms', 'seq']
            for jn in JOINT_ORDER:
                hdr += [f'{jn}_pos_deg', f'{jn}_vel_deg_s', f'{jn}_tau_Nm', f'{jn}_temp_C', f'{jn}_fault']
            fcsv.write(','.join(hdr) + '\n')

        t_start = time.monotonic()
        t_next_print = t_start + 1.0
        try:
            while True:
                if self.args.duration and (time.monotonic() - t_start) >= self.args.duration:
                    break
                try:
                    pkt, _ = s.recvfrom(2048)
                except socket.timeout:
                    if self.args.duration is None and self.n_recv == 0:
                        print("[MOSS] 1 s 内未收到任何帧 —— 检查网线/下位机是否在上报/端口是否 5001")
                    continue
                arr = time.time_ns()
                vals = self.handle(pkt, arr)
                if vals is None:
                    continue
                fbin.write(pkt)
                farr.write(struct.pack('<Q', arr))
                if fcsv:
                    seq = vals[2]
                    ts = vals[3]
                    row = [str(arr), str(ts), str(seq)]
                    for j in range(6):
                        off = 8 + j * 17
                        jid, pos, vel, tau, temp, fault = struct.unpack_from(JOINT_STATUS_FMT, pkt, off)
                        row += [f'{pos/1000.0:.3f}', f'{vel/1000.0:.3f}', f'{tau/1000.0:.4f}',
                                f'{temp/10.0:.1f}', str(fault)]
                    fcsv.write(','.join(row) + '\n')
                now = time.monotonic()
                if now >= t_next_print:
                    t_next_print = now + 1.0
                    el = now - t_start
                    print(f"  t={el:6.1f}s  帧={self.n_valid:7d}  {self.n_valid/max(el,1e-9):7.1f} Hz  "
                          f"缺口={self.n_missing:5d}  CRC错={self.err_crc}  长度错={self.err_size}  "
                          f"|最大位置|={max(abs(x) for x in self.jpos_max if x is not None):.1f}°")
        except KeyboardInterrupt:
            print("\n[MOSS] 手动停止。")
        finally:
            fbin.flush(); fbin.close()
            farr.flush(); farr.close()
            if fcsv:
                fcsv.flush(); fcsv.close()
            s.close()

        st = self.stats()
        meta = {
            'created': datetime.now(timezone(timedelta(hours=8))).isoformat(),
            'tool': 'collect_motion_data.py (零固件改动采集器)',
            'label': self.args.label,
            'board': {'model': 'STM32F407 + LwIP', 'mode': 'ZERO (零阻力)',
                      'other_modes': '按方案 §2 应硬禁用', 'report_rate_hz': 1000,
                      'udp_port': self.port},
            'channels': {
                'joint_order': JOINT_ORDER, 'joint_bus': JOINT_BUS, 'joint_motor': JOINT_MOTOR,
                'fields': [
                    {'offset': 0, 'name': 'frame_type', 'type': 'uint8'},
                    {'offset': 1, 'name': 'reserved', 'type': 'uint8'},
                    {'offset': 2, 'name': 'seq_num', 'type': 'uint16'},
                    {'offset': 4, 'name': 'timestamp_ms', 'type': 'uint32', 'note': 'HAL_GetTick(), 板上时钟'},
                ] + [
                    {'offset': 8 + i * 17, 'name': f'joint{i}', 'type': '17B',
                     'detail': 'uint8 joint_id, int32 position(mdeg), int32 velocity(mdeg/s), '
                               'int32 torque(mNm), uint16 temperature(0.1C), uint16 fault_code'}
                    for i in range(6)
                ],
                'record_size': REPORT_FRAME_SIZE,
                'crc': 'CRC16 MODBUS, 覆盖前 110 字节',
            },
            'session_metadata': {
                'subject_id': self.args.subject,
                'subject_mass_kg': self.args.subject_mass,
                'load_mass_kg': self.args.load,
                'load_position': self.args.load_pos,
                'mount_note': self.args.mount,
                'theta0_deg': self.args.theta0,
                'operator': self.args.operator,
                'note': self.args.note,
            },
            'stats': st,
            'artifacts': {'bin': os.path.basename(self.f_bin), 'arr': os.path.basename(self.f_arr),
                          'csv': os.path.basename(self.f_csv) if self.f_csv else None},
            'qc_targets': {'jitter_ms': '<1', 'loss_rate_pct': '<0.1', 'tau_cmd_zero': 'ZERO 模式下应恒零',
                           'tau_saturation': '0'},
        }
        with open(self.f_json, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print("\n===== 质控 =====")
        for k, v in st.items():
            if k != 'gap_events_first20':
                print(f"  {k:24s}: {v}")
        if st['seq_gap_events']:
            print(f"  gap_events(first20)      : {st['gap_events_first20']}")
        sat = {JOINT_ORDER[j]: c for j, c in enumerate(self.tau_sat) if c}
        print(f"  力矩饱和帧数            : {sat if sat else '无'}")
        print("  各关节位置范围(deg)     :")
        for j in range(6):
            if self.jpos_min[j] is not None:
                print(f"    {JOINT_ORDER[j]:8s} [{self.jpos_min[j]:8.2f}, {self.jpos_max[j]:8.2f}]  "
                      f"τ[{self.tau_min[j]:7.2f}, {self.tau_max[j]:7.2f}] N·m  T={self.temperature[j]:.1f}°C")
        verdict = (st['loss_rate_pct'] < 0.1 and st['err_crc'] == 0 and st['err_size'] == 0
                   and abs(st['rate_hz'] - 1000) < 30
                   and st['coverage_pct'] >= 99.9
                   and st['board_max_dt_ms'] <= self.args.gap_ms)
        if len(self.dt_big) > 5:
            worst = sorted(self.dt_big, key=lambda x: -x[1])[:5]
            print(f"  ★ 最严重的 {len(worst)} 个时间空洞 (board_ts_ms, dt_ms): {worst}")
            print("     ↑ 若呈【每秒一次、数十 ms】规律 → 即 control_isr.c:1000-1027 的 ISR 内 printf 阻塞")
        print(f"\n[MOSS] 判定: {'通过 (达标)' if verdict else '★ 不达标 —— 按方案 §10 需重采'}")
        print(f"[MOSS] 已写出: {self.f_json}")
        return 0 if verdict else 2


def main():
    ap = argparse.ArgumentParser(description='外骨骼人体运动数据采集器 (零固件改动)')
    ap.add_argument('--label', required=True, help='工况标签, 如 G0_baseline / G1_LHip_1.0Hz')
    ap.add_argument('--duration', type=float, default=None, help='采集时长(s); 省略则 Ctrl-C 停止')
    ap.add_argument('--port', type=int, default=UDP_PORT_DEFAULT)
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--outdir', default='data')
    ap.add_argument('--rcvbuf', type=int, default=8 * 1024 * 1024, help='SO_RCVBUF 字节')
    ap.add_argument('--gap-ms', type=float, default=5.0,
                    help='板上时间戳间隔超过该值即记为"时间空洞"(默认 5ms)')
    ap.add_argument('--csv', action='store_true', help='额外写出解码 CSV (体积大)')
    # 方案 §8 元数据
    ap.add_argument('--subject', default='', help='受试者 ID')
    ap.add_argument('--subject-mass', type=float, default=None, help='受试者体重 kg')
    ap.add_argument('--load', type=float, default=None, help='附加负载质量 kg (0=空载)')
    ap.add_argument('--load-pos', default='', help='负载位置/握点, 如 "右手, r=0.35m"')
    ap.add_argument('--mount', default='', help='穿戴/装配姿态说明')
    ap.add_argument('--theta0', type=float, default=None, help='自由下垂零位 θ0 (deg)')
    ap.add_argument('--operator', default='', help='操作者')
    ap.add_argument('--note', default='', help='备注')
    ap.add_argument('--selftest', action='store_true', help='校验 CRC 表驱动实现后退出')
    args = ap.parse_args()

    if args.selftest:
        import random
        ok = True
        for _ in range(2000):
            d = bytes(random.getrandbits(8) for _ in range(random.randint(1, 200)))
            if crc16(d) != crc16_naive(d):
                ok = False
                break
        print('[selftest] CRC16 表驱动 == 逐位实现:', 'PASS' if ok else 'FAIL')
        return 0 if ok else 1

    return Collector(args).run()


if __name__ == '__main__':
    sys.exit(main())
