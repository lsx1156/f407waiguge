#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F407 SD 卡日志提取 / 校验 / 导出工具
====================================

读取板上写出的 LOG_nnnn.BIN，校验文件头，导出 CSV / 统计。
记录格式 = 与 UDP 上报帧完全相同的 112 B ReportFrame_t（见 App/bsp/contract.h）。

用法:
    python sd_log_extract.py LOG_0001.BIN                 # 校验 + 打印统计
    python sd_log_extract.py LOG_0001.BIN --csv out.csv   # 另导出解码 CSV
    python sd_log_extract.py LOG_0001.BIN --json out.json
    python sd_log_extract.py --selftest                   # 自测(造一个合成日志再解析)

文件布局 (设计 docs/SD卡采集子系统设计_v1.md §3):
    [0x000] 512 B 文件头 (magic "F407LOG1")
    [0x200] N × 112 B 记录   ← N 由头里的 rec_total 给出 (权威)
"""
import argparse
import json
import os
import struct
import sys

REC_SIZE = 112
HDR_SIZE = 512
MAGIC = b'F407LOG1'

JOINT_ORDER = ['L-Hip', 'R-Hip', 'L-Shldr', 'L-Elbow', 'R-Shldr', 'R-Elbow']

# 文件头: 前 60 B (packed) — 8s + H + H + I*6 + 16s + I*2 = 12 字段
HDR_FMT = '<8sHHIIIIII16sII'
HDR_FIELDS = ['magic', 'ver', 'rec_size', 'sample_rate', 'blk_recs',
              'rec_total', 'rec_dropped', 'ts_first', 'ts_last',
              'fw', 'blk_flushed', 'write_err']

REPORT_FMT = '<' + 'BBHI' + 'BiiiHH' * 6 + 'H'


def parse_header(buf: bytes) -> dict:
    vals = struct.unpack('<8sHHIIIIII16sII', buf[:60])
    h = dict(zip(HDR_FIELDS, vals))
    h['magic'] = h['magic'].rstrip(b'\x00').decode('ascii', 'replace')
    h['fw'] = h['fw'].rstrip(b'\x00').decode('ascii', 'replace')
    return h


def crc_table():
    t = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
        t.append(c)
    return t


_CRC = crc_table()


def crc16(data: bytes) -> int:
    c = 0xFFFF
    for b in data:
        c = (c >> 8) ^ _CRC[(c ^ b) & 0xFF]
    return c


def extract(path, csv_path=None, json_path=None):
    size = os.path.getsize(path)
    with open(path, 'rb') as f:
        hdr_raw = f.read(HDR_SIZE)
        if len(hdr_raw) < HDR_SIZE:
            print(f'[FAIL] 文件小于 {HDR_SIZE} B，头部不完整'); return 2
        h = parse_header(hdr_raw)
        if h['magic'] != 'F407LOG1':
            print(f"[FAIL] magic 不符: {h['magic']!r} (期望 'F407LOG1')"); return 2

        n = h['rec_total']
        body_bytes = min(n * REC_SIZE, size - HDR_SIZE)
        n_avail = body_bytes // REC_SIZE
        data = f.read(n_avail * REC_SIZE)

    ok = True
    print('===== 文件头 =====')
    for k in ('magic', 'ver', 'rec_size', 'sample_rate', 'blk_recs',
              'rec_total', 'rec_dropped', 'ts_first', 'ts_last',
              'fw', 'blk_flushed', 'write_err'):
        print(f'  {k:14s}: {h[k]}')
    print(f'  {"文件字节":14s}: {size}')
    if h['rec_size'] != REC_SIZE:
        print(f"  ★ rec_size={h['rec_size']} != {REC_SIZE}"); ok = False
    if n_avail < n:
        print(f"  ★ 文件实际只有 {n_avail} 条记录, 头声称 {n} 条 (断电截断?)" ); ok = False

    recs = []
    ts_gaps = 0
    seq_gaps = 0
    prev_ts = prev_seq = None
    jmin = [None] * 6; jmax = [None] * 6
    tmin = [None] * 6; tmax = [None] * 6
    bad_crc = 0
    for i in range(n_avail):
        pkt = data[i * REC_SIZE:(i + 1) * REC_SIZE]
        if crc16(pkt[:-2]) != struct.unpack_from('<H', pkt, REC_SIZE - 2)[0]:
            bad_crc += 1
        v = struct.unpack(REPORT_FMT, pkt)
        seq, ts = v[2], v[3]
        if prev_ts is not None:
            d = (ts - prev_ts) & 0xFFFFFFFF
            if d > 5:
                ts_gaps += 1
        if prev_seq is not None:
            d = (seq - prev_seq) & 0xFFFF
            if d > 1:
                seq_gaps += d - 1
        prev_ts, prev_seq = ts, seq
        row = []
        for j in range(6):
            off = 8 + j * 17
            ji, pos, vel, tau, temp, fault = struct.unpack_from('<BiiiHH', pkt, off)
            pd, td = pos / 1000.0, tau / 1000.0
            jmin[j] = pd if jmin[j] is None or pd < jmin[j] else jmin[j]
            jmax[j] = pd if jmax[j] is None or pd > jmax[j] else jmax[j]
            tmin[j] = td if tmin[j] is None or td < tmin[j] else tmin[j]
            tmax[j] = td if tmax[j] is None or td > tmax[j] else tmax[j]
            row += [pd, vel / 1000.0, td, temp / 10.0, fault]
        recs.append((ts, seq, row))

    dur = 0.0
    if recs:
        dur = ((recs[-1][0] - recs[0][0]) & 0xFFFFFFFF) / 1000.0
    rate = (len(recs) / dur) if dur > 0 else 0.0
    cov = 100.0 * len(recs) / max(1.0, dur * 1000.0 + 1.0)

    print('\n===== 统计 =====')
    print(f'  记录数(解析)     : {len(recs)}  (头声称 {n})')
    print(f'  时长             : {dur:.3f} s')
    print(f'  实际速率         : {rate:.2f} Hz  (期望 1000)')
    print(f'  覆盖率           : {cov:.3f} %    ← 断电前的空洞会被这里抓到')
    print(f'  时间空洞(>5ms)   : {ts_gaps}')
    print(f'  序号缺口(丢帧)   : {seq_gaps}')
    print(f'  CRC 错误记录     : {bad_crc}')
    print(f'  头报告丢弃       : {h["rec_dropped"]}   写错误: {h["write_err"]}')
    print('  各关节范围(deg) / 力矩(N·m):')
    for j in range(6):
        if jmin[j] is not None:
            print(f'    {JOINT_ORDER[j]:8s} θ[{jmin[j]:8.2f},{jmax[j]:8.2f}]  τ[{tmin[j]:7.2f},{tmax[j]:7.2f}]')

    verdict = (bad_crc == 0 and n_avail >= n and abs(rate - 1000) < 30 and cov >= 99.0)
    print(f'\n[判定] {"通过" if verdict else "★ 不达标/需检查"}')

    if csv_path and recs:
        hdr_cols = ['ts_ms', 'seq']
        for jn in JOINT_ORDER:
            hdr_cols += [f'{jn}_pos_deg', f'{jn}_vel_deg_s', f'{jn}_tau_Nm', f'{jn}_temp_C', f'{jn}_fault']
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.write(','.join(hdr_cols) + '\n')
            for ts, seq, row in recs:
                cells = [str(ts), str(seq)] + [f'{x:.4f}' if isinstance(x, float) else str(x) for x in row]
                f.write(','.join(cells) + '\n')
        print(f'[写出] CSV: {csv_path}')

    if json_path:
        meta = {'file': os.path.basename(path), 'header': {k: h[k] for k in HDR_FIELDS},
                'parsed_records': len(recs), 'duration_s': dur, 'rate_hz': rate,
                'coverage_pct': cov, 'time_gaps': ts_gaps, 'seq_gaps': seq_gaps,
                'crc_errors': bad_crc, 'verdict': 'PASS' if verdict else 'FAIL'}
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        print(f'[写出] JSON: {json_path}')

    return 0 if verdict else 2


def selftest():
    """造一个合成 LOG 再解析, 验证头部/记录/统计链路 (含一个 43ms 空洞)"""
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), 'sdlog_selftest.bin')
    n = 3000
    hdr = bytearray(HDR_SIZE)
    struct.pack_into('<8sHHIIIIII16sII', hdr, 0, MAGIC, 0x0202, REC_SIZE, 1000, 1024,
                     n, 0, 0, n - 1, b'v2.2.0-sdlog', 3, 0)
    with open(tmp, 'wb') as f:
        f.write(hdr)
        board = 0
        seq = 0
        for i in range(n):
            if (board % 1000) < 43:          # 注入每秒 43ms 空洞
                board += 43
            seq = (seq + 1) & 0xFFFF
            h = struct.pack('<BBHI', 1, 0, seq, board)
            b = b''.join(struct.pack('<BiiiHH', j + 1, i, 1000, 250 + j, 0, 0) for j in range(6))
            pkt = h + b
            pkt += struct.pack('<H', crc16(pkt))
            f.write(pkt)
            board += 1
    print(f'[selftest] 合成 {tmp} ({os.path.getsize(tmp)} B)')
    rc = extract(tmp, csv_path=None)
    print(f'[selftest] extract 返回 {rc} (注入空洞 → 期望 2=不达标)')
    return 0 if rc == 2 else 1


def main():
    ap = argparse.ArgumentParser(description='F407 SD 卡日志提取/校验')
    ap.add_argument('file', nargs='?', help='LOG_nnnn.BIN')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--json', default=None)
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.file:
        ap.print_help(); return 1
    return extract(a.file, a.csv, a.json)


if __name__ == '__main__':
    sys.exit(main())
