#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F407 数据/日志自动化采集流水线
==============================

按《数据采集方案_零阻力台架穿戴_v1.md》§7 的工况矩阵, 自动依次:
    ① 播报工况指令 → ② 倒计时 → ③ 采 UDP 运动数据(1kHz) → ④ (可选)同步抓串口日志
    → ⑤ 质控判定 → ⑥ 写会话索引 →  失败的工况标记待重采

用法:
    python auto_acquire.py --dry-run                 # 只打印计划, 不采
    python auto_acquire.py --only G0,G7              # 只采指定工况
    python auto_acquire.py --serial                  # 同时抓串口日志
    python auto_acquire.py --mock                    # 无硬件自测(内置 1kHz 模拟下位机)
    python auto_acquire.py --resume                  # 跳过索引里已通过的工况
    python auto_acquire.py --countdown 10 --subject S1

产出 (outdir 默认 data, 日志默认 logs):
    matrix.json                    当前工况矩阵(首次运行自动生成, 可手改)
    sessions_index.json            本次会话索引: 每个工况的 QC + 文件 + 通过/失败
    <collect_motion_data.py 的产物>
"""

import argparse
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
import types
from datetime import datetime, timezone, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import collect_motion_data as C   # noqa: E402

# ---------------------------------------------------------------------------
# 默认工况矩阵 (方案 §7 的精编首轮版本; --full 用全量)
# ---------------------------------------------------------------------------
def default_matrix(full=False):
    def c(cid, label, dur, instr, reps=1, load=None, joint=None, freq=None):
        return {'id': cid, 'label': label, 'duration_s': dur, 'instruction': instr,
                'reps': reps, 'load_kg': load, 'joint': joint, 'freq_hz': freq}

    m = [
        c('G0', 'G0_baseline', 30, '受试者完全放松, 装置自由下垂, 不要用力', reps=3),
        c('G1', 'G1_LHip_1.0Hz', 20, '主动摆动【左髋】, 跟着节拍器约 1.0 Hz, 中等幅度', joint='L-Hip', freq=1.0),
        c('G1', 'G1_LHip_2.0Hz', 20, '主动摆动【左髋】, 约 2.0 Hz(较快), 中等幅度', joint='L-Hip', freq=2.0),
        c('G1', 'G1_RHip_1.0Hz', 20, '主动摆动【右髋】, 约 1.0 Hz, 中等幅度', joint='R-Hip', freq=1.0),
        c('G1', 'G1_LShldr_1.0Hz', 20, '主动摆动【左肩】, 约 1.0 Hz, 中等幅度', joint='L-Shldr', freq=1.0),
        c('G1', 'G1_LShldr_2.0Hz', 20, '主动摆动【左肩】, 约 2.0 Hz, 中等幅度', joint='L-Shldr', freq=2.0),
        c('G1', 'G1_LElbow_1.0Hz', 20, '主动屈伸【左肘】, 约 1.0 Hz, 全范围', joint='L-Elbow', freq=1.0),
        c('G1', 'G1_RShldr_1.0Hz', 20, '主动摆动【右肩】, 约 1.0 Hz, 中等幅度', joint='R-Shldr', freq=1.0),
        c('G5', 'G5_passive_sweep_LHip', 10, '人放松! 由操作者手扶缓慢来回转动【左髋】0.05→3 rad/s', joint='L-Hip'),
        c('G5', 'G5_passive_sweep_LShldr', 10, '人放松! 由操作者手扶缓慢来回转动【左肩】', joint='L-Shldr'),
        c('G5', 'G5_passive_sweep_LElbow', 10, '人放松! 由操作者手扶缓慢来回转动【左肘】', joint='L-Elbow'),
        c('G7', 'G7_torque_check_LShldr', 10, '【校核】左肩水平位挂已知质量(记录数值), 静态保持不动', joint='L-Shldr'),
        c('G7', 'G7_torque_check_LElbow', 10, '【校核】左肘挂已知质量(记录数值), 静态保持不动', joint='L-Elbow'),
        c('G3', 'G3_upper_cycle', 60, '上肢往复作业: 抓取-搬运-放置, 自然节奏', reps=2),
        c('G4', 'G4_upper_2kg', 20, '手持 2 kg(记录握点距离), 做与 G3 相同的往复动作', load=2.0),
        c('G4', 'G4_upper_5kg', 20, '手持 5 kg(记录握点距离), 做与 G3 相同的往复动作', load=5.0),
    ]
    if full:
        m = ([m[0]] +
             [c('G1', f'G1_{j}_{f}Hz_{a}', 20, f'主动摆动【{j}】, 约 {f} Hz, 幅度{a}',
                joint=j, freq=f)
              for j in ['L-Hip', 'R-Hip', 'L-Shldr', 'L-Elbow', 'R-Shldr', 'R-Elbow']
              for f in [0.5, 1.0, 1.5, 2.0, 2.5]
              for a in ['小', '中', '大']] + m[1:])
    return m


# ---------------------------------------------------------------------------
# 内置模拟下位机 (--mock, 用于无硬件时验证整条流水线)
# ---------------------------------------------------------------------------
class MockBoard(threading.Thread):
    def __init__(self, port, stall_ms=0):
        super().__init__(daemon=True)
        self.port, self.stall_ms = port, stall_ms
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        board_ms = 0
        seq = 0
        import math
        t0 = time.perf_counter()
        while not self._stop.is_set():
            if self.stall_ms and (board_ms % 1000) < self.stall_ms:
                pass                                   # 模拟 ISR 内 printf 阻塞
            else:
                seq = (seq + 1) & 0xFFFF
                t = board_ms / 1000.0
                hdr = struct.pack('<BBHI', 0x01, 0, seq, board_ms & 0xFFFFFFFF)
                body = b''
                for j in range(6):
                    ph = 2 * math.pi * 1.0 * t + j * 0.5
                    body += struct.pack('<BiiiHH', j + 1,
                                        int(20000 * math.sin(ph)), int(125663 * math.cos(ph)),
                                        int(3000 * math.sin(ph)), 250 + j, 0)
                pkt = hdr + body
                pkt += struct.pack('<H', C.crc16(pkt))
                s.sendto(pkt, ('127.0.0.1', self.port))
            board_ms += 1
            d = (t0 + board_ms / 1000.0) - time.perf_counter()
            if d > 0:
                time.sleep(d)
        s.close()


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='F407 数据/日志自动化采集流水线')
    ap.add_argument('--outdir', default='data')
    ap.add_argument('--logdir', default='logs')
    ap.add_argument('--matrix', default=None, help='工况矩阵 JSON (默认 <outdir>/matrix.json)')
    ap.add_argument('--full', action='store_true', help='用方案 §7 全量矩阵(90+ 工况)')
    ap.add_argument('--only', default=None, help='只采这些工况 id/标签, 逗号分隔, 如 G0,G7')
    ap.add_argument('--resume', action='store_true', help='跳过索引中已通过的工况')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划')
    ap.add_argument('--serial', action='store_true', help='同时抓串口日志')
    ap.add_argument('--serial-port', default='auto')
    ap.add_argument('--countdown', type=int, default=5, help='每个工况前的准备倒计时(s)')
    ap.add_argument('--gap-ms', type=float, default=5.0)
    ap.add_argument('--port', type=int, default=5001)
    ap.add_argument('--subject', default='')
    ap.add_argument('--subject-mass', type=float, default=None)
    ap.add_argument('--mount', default='')
    ap.add_argument('--operator', default='')
    ap.add_argument('--mock', action='store_true', help='内置模拟下位机(无硬件自测)')
    ap.add_argument('--mock-stall-ms', type=float, default=0.0,
                    help='模拟 ISR 内 printf 阻塞(如 43); 用于演示质控能抓到')
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    os.makedirs(args.logdir, exist_ok=True)

    # 矩阵: 文件优先, 否则默认并写盘
    mpath = args.matrix or os.path.join(args.outdir, 'matrix.json')
    if os.path.exists(mpath):
        with open(mpath, encoding='utf-8') as f:
            matrix = json.load(f)
        print(f'[MOSS] 载入矩阵 {mpath} ({len(matrix)} 个工况)')
    else:
        matrix = default_matrix(args.full)
        with open(mpath, 'w', encoding='utf-8') as f:
            json.dump(matrix, f, ensure_ascii=False, indent=2)
        print(f'[MOSS] 生成默认矩阵 → {mpath} ({len(matrix)} 个工况)')

    # --only 过滤
    if args.only:
        want = {s.strip() for s in args.only.split(',')}
        matrix = [c for c in matrix if c['id'] in want or c['label'] in want]
        print(f'[MOSS] --only 过滤后 {len(matrix)} 个工况')

    # 展开 reps → 计划 (必须在 --resume 过滤之前, 否则多次重复工况的标签对不上)
    plan = []
    for c in matrix:
        for r in range(1, int(c.get('reps', 1)) + 1):
            e = dict(c)
            if e.get('reps', 1) > 1:
                e['label'] = f"{c['label']}_r{r}"
            e['rep'] = r
            plan.append(e)

    # --resume: 跳过已通过 (按展开后的实际标签比对)
    idxpath = os.path.join(args.outdir, 'sessions_index.json')
    if args.resume and os.path.exists(idxpath):
        with open(idxpath, encoding='utf-8') as f:
            old = json.load(f)
        passed = {r['label'] for r in old.get('records', []) if r.get('verdict') == 'PASS'}
        before = len(plan)
        plan = [e for e in plan if e['label'] not in passed]
        print(f'[MOSS] --resume: 跳过 {before - len(plan)} 个已通过工况')
        if not plan:
            print('[MOSS] 所有工况均已通过, 无需采集。')
            return 0

    total_s = sum(e['duration_s'] for e in plan)
    print(f"[MOSS] 本次计划 {len(plan)} 个工况, 预计采集 {total_s/60:.1f} min"
          f" (不含准备/休息)")
    for i, e in enumerate(plan, 1):
        print(f"  {i:2d}. [{e['id']:2s}] {e['label']:28s} {e['duration_s']:>4}s  {e['instruction']}")
    if args.dry_run:
        print('[MOSS] --dry-run: 未开始采集。')
        return 0

    mock = MockBoard(args.port, args.mock_stall_ms) if args.mock else None
    if mock:
        mock.start()
        time.sleep(0.5)
        print(f'[MOSS] 模拟下位机已启动 → 127.0.0.1:{args.port}'
              + (f' (注入 ISR 阻塞 {args.mock_stall_ms:.0f} ms/s)' if args.mock_stall_ms else ''))

    records = []
    t_session = time.monotonic()
    try:
        for i, e in enumerate(plan, 1):
            print(f"\n{'='*72}\n[{i}/{len(plan)}] {e['label']}   (id={e['id']}, {e['duration_s']}s)")
            print(f"  动作要求: {e['instruction']}")
            if e.get('load_kg'):
                print(f"  ★ 负载: {e['load_kg']} kg —— 记录握点/绑点距离, 结束后填入 --load-pos")
            for s in range(args.countdown, 0, -1):
                print(f"  准备… {s}", end='\r', flush=True)
                time.sleep(1)
            print('  开始采集!              ')

            ser_proc = None
            if args.serial:
                slog = os.path.join(args.logdir, f'{e["label"]}_serial.log')
                ser_proc = subprocess.Popen(
                    [sys.executable, os.path.join(HERE, 'f407_serial_log.py'),
                     '--port', args.serial_port, '--out', slog,
                     '--duration', str(e['duration_s'])])

            ns = types.SimpleNamespace(
                label=e['label'], duration=e['duration_s'], port=args.port, host='0.0.0.0',
                outdir=args.outdir, rcvbuf=8 * 1024 * 1024, csv=False, gap_ms=args.gap_ms,
                subject=args.subject, subject_mass=args.subject_mass,
                load=e.get('load_kg'), load_pos='', mount=args.mount,
                theta0=None, operator=args.operator, note=e['instruction'],
            )
            col = C.Collector(ns)
            rc = col.run()
            st = col.stats()

            if ser_proc:
                ser_proc.wait(timeout=10)

            verdict = 'PASS' if rc == 0 else 'FAIL'
            rec = {'label': e['label'], 'id': e['id'], 'duration_s': e['duration_s'],
                   'instruction': e['instruction'], 'load_kg': e.get('load_kg'),
                   'joint': e.get('joint'), 'freq_hz': e.get('freq_hz'),
                   'verdict': verdict, 'stats': st,
                   'files': {'bin': col.f_bin, 'json': col.f_json, 'arr': col.f_arr,
                             'serial': (os.path.join(args.logdir, f'{e["label"]}_serial.log')
                                        if args.serial else None)}}
            records.append(rec)
            print(f"  → {verdict}  ({st['rate_hz']} Hz, coverage {st['coverage_pct']}%, "
                  f"丢帧 {st['seq_missing_frames']}, 最大空洞 {st['board_max_dt_ms']} ms)")
            if verdict == 'FAIL':
                print(f"  ★ 该工况需重采: python auto_acquire.py --only {e['label']} --resume")
            if i < len(plan):
                print('  休息 10 s…')
                time.sleep(10)
    except KeyboardInterrupt:
        print('\n[MOSS] 用户中断, 已采部分仍会写索引。')
    finally:
        if mock:
            mock.stop()

    npass = sum(1 for r in records if r['verdict'] == 'PASS')
    index = {
        'created': datetime.now(timezone(timedelta(hours=8))).isoformat(),
        'tool': 'auto_acquire.py',
        'session': {'subject': args.subject, 'subject_mass_kg': args.subject_mass,
                    'mount': args.mount, 'operator': args.operator,
                    'countdown_s': args.countdown, 'gap_ms': args.gap_ms,
                    'mock': bool(args.mock), 'serial': bool(args.serial),
                    'elapsed_min': round((time.monotonic() - t_session) / 60.0, 2)},
        'summary': {'total': len(records), 'pass': npass, 'fail': len(records) - npass},
        'records': records,
    }
    with open(idxpath, 'w', encoding='utf-8') as f:
        json.dump(index, f, ensure_ascii=False, indent=2)

    print(f"\n{'='*72}\n===== 会话汇总 =====")
    for r in records:
        mark = '✓' if r['verdict'] == 'PASS' else '✗'
        print(f"  {mark} {r['label']:30s} {r['stats']['rate_hz']:>7.1f} Hz  "
              f"coverage {r['stats']['coverage_pct']:>6.2f}%  空洞 {r['stats']['board_max_dt_ms']:>5.1f} ms")
    print(f"\n  通过 {npass}/{len(records)}   索引: {idxpath}")
    if npass < len(records):
        print('  重采失败项: python auto_acquire.py --resume')
    return 0 if npass == len(records) else 2


if __name__ == '__main__':
    sys.exit(main())
