#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
F407 串口日志采集器 (零依赖: 直接调 Win32 API, 不需要 pyserial)
================================================================

用途: 自动抓取 F407 的 USART1 (115200 8N1) 打印日志, 带时间戳落盘。
      典型内容: [JOINT] 6 joints registered / [CAN2] RS01 summary: 4/4 online /
                [TIM6] Init OK: 1ms tick / 故障码 / 模式切换。

用法:
    python f407_serial_log.py --list                    # 列出串口
    python f407_serial_log.py --port auto --probe 5     # 只探测能否打开/收数
    python f407_serial_log.py --port COM3 --duration 60 # 采 60 s
    python f407_serial_log.py --port COM3 --out logs/boot.log

自测 (无串口硬件时验证落盘路径):
    python f407_serial_log.py --selftest

退出码: 0=正常, 1=参数/环境错, 3=打开串口失败
"""

import argparse
import ctypes
import ctypes.wintypes as wt
import os
import sys
import time
from datetime import datetime

# ---------------------------------------------------------------------------
# Win32 串口 (ctypes)
# ---------------------------------------------------------------------------
GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wt.HANDLE(-1).value
PURGE_TXCLEAR = 0x0004
PURGE_RXCLEAR = 0x0008
ERR_ACCESS_DENIED = 5
ERR_FILE_NOT_FOUND = 2

# DCB.Flags 位: fBinary | fDtrControl=1 | fTXContinueOnXoff | fRtsControl=1
DCB_FLAGS_8N1 = 0x0001 | 0x0010 | 0x0080 | 0x1000


class DCB(ctypes.Structure):
    _fields_ = [
        ('DCBlength', wt.DWORD), ('BaudRate', wt.DWORD), ('Flags', wt.DWORD),
        ('wReserved', wt.WORD), ('XonLim', wt.WORD), ('XoffLim', wt.WORD),
        ('ByteSize', ctypes.c_ubyte), ('Parity', ctypes.c_ubyte), ('StopBits', ctypes.c_ubyte),
        ('XonChar', ctypes.c_char), ('XoffChar', ctypes.c_char),
        ('ErrorChar', ctypes.c_char), ('EofChar', ctypes.c_char), ('EvtChar', ctypes.c_char),
        ('wReserved1', wt.WORD),
    ]


class COMMTIMEOUTS(ctypes.Structure):
    _fields_ = [('ReadIntervalTimeout', wt.DWORD), ('ReadTotalTimeoutMultiplier', wt.DWORD),
                ('ReadTotalTimeoutConstant', wt.DWORD), ('WriteTotalTimeoutMultiplier', wt.DWORD),
                ('WriteTotalTimeoutConstant', wt.DWORD)]


def list_ports():
    """从注册表枚举当前存在的 COM 口"""
    import winreg
    out = []
    try:
        k = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM')
        i = 0
        while True:
            try:
                _n, v, _t = winreg.EnumValue(k, i)
                out.append(v)
                i += 1
            except OSError:
                break
    except FileNotFoundError:
        pass
    return sorted(out, key=lambda s: (len(s), s))


class Win32Serial:
    """阻塞式串口读取 (ReadIntervalTimeout=MAX 且总超时=0 → 立即返回已有数据)"""

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.k32 = ctypes.WinDLL('kernel32', use_last_error=True)
        self.h = None

    def open(self):
        path = f'\\\\.\\{self.port}'
        self.h = self.k32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE, 0, None,
                                      OPEN_EXISTING, 0, None)
        if self.h == INVALID_HANDLE_VALUE or self.h is None or self.h == 0:
            err = ctypes.get_last_error()
            if err == ERR_ACCESS_DENIED:
                raise OSError(f'{self.port} 被占用 (ERROR_ACCESS_DENIED) —— 关掉其它串口工具再试')
            if err == ERR_FILE_NOT_FOUND:
                raise OSError(f'{self.port} 不存在 (ERROR_FILE_NOT_FOUND)')
            raise OSError(f'打开 {self.port} 失败, Win32 错误码 {err}')

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not self.k32.GetCommState(self.h, ctypes.byref(dcb)):
            err = ctypes.get_last_error()
            self.close()
            raise OSError(f'GetCommState 失败, 错误码 {err}')
        dcb.BaudRate = self.baud
        dcb.ByteSize = 8
        dcb.Parity = 0        # NOPARITY
        dcb.StopBits = 0      # ONESTOPBIT
        dcb.Flags = DCB_FLAGS_8N1
        if not self.k32.SetCommState(self.h, ctypes.byref(dcb)):
            err = ctypes.get_last_error()
            self.close()
            raise OSError(f'SetCommState 失败 (波特率 {self.baud} 可能不被支持), 错误码 {err}')

        to = COMMTIMEOUTS()
        to.ReadIntervalTimeout = 0xFFFFFFFF
        to.ReadTotalTimeoutMultiplier = 0
        to.ReadTotalTimeoutConstant = 0
        to.WriteTotalTimeoutMultiplier = 0
        to.WriteTotalTimeoutConstant = 0
        if not self.k32.SetCommTimeouts(self.h, ctypes.byref(to)):
            err = ctypes.get_last_error()
            self.close()
            raise OSError(f'SetCommTimeouts 失败, 错误码 {err}')

        self.k32.PurgeComm(self.h, PURGE_RXCLEAR | PURGE_TXCLEAR)
        return True

    def read(self, n=4096):
        if not self.h:
            return b''
        buf = ctypes.create_string_buffer(n)
        got = wt.DWORD(0)
        ok = self.k32.ReadFile(self.h, buf, n, ctypes.byref(got), None)
        if not ok:
            return b''
        return buf.raw[:got.value]

    def close(self):
        if self.h:
            self.k32.CloseHandle(self.h)
            self.h = None


# ---------------------------------------------------------------------------
# 日志写入
# ---------------------------------------------------------------------------
class Logger:
    def __init__(self, out, stamp=True, rotate_mb=0):
        os.makedirs(os.path.dirname(os.path.abspath(out)) or '.', exist_ok=True)
        self.out = out
        self.stamp = stamp
        self.rotate_mb = rotate_mb
        self.f = open(out, 'w', encoding='utf-8', errors='replace', buffering=1)
        self.t0 = time.monotonic()
        self.nlines = 0
        self.nbytes = 0
        self.buf = b''

    def feed(self, data: bytes):
        """按行解析并按需加时间戳"""
        if not data:
            return
        self.buf += data
        while b'\n' in self.buf:
            raw, self.buf = self.buf.split(b'\n', 1)
            line = raw.decode('utf-8', 'replace').rstrip('\r')
            if not line:
                continue
            self.line(line)

    def line(self, text: str):
        el = time.monotonic() - self.t0
        s = f'[{el:9.3f}s] {text}' if self.stamp else text
        self.f.write(s + '\n')
        self.nlines += 1
        self.nbytes += len(s) + 1

    def close(self):
        if self.buf:
            self.line(self.buf.decode('utf-8', 'replace'))
            self.buf = b''
        self.f.flush()
        self.f.close()


# ---------------------------------------------------------------------------
def cmd_selftest():
    """无串口硬件时验证: 落盘/时间戳/按行切分"""
    import tempfile
    tmp = os.path.join(tempfile.gettempdir(), 'serial_selftest.log')
    lg = Logger(tmp)
    lg.feed(b'[TIM6] Init OK: 1ms tick\r\n[JOINT] 6 joints regis')
    time.sleep(0.05)
    lg.feed(b'tered: [0]L-Hip(b0,id0x01,ESO=1)\r\n[CAN2] RS01 summary: 4/4 motors online\r\n')
    time.sleep(0.05)
    lg.feed(b'partial line no newline')
    lg.close()
    with open(tmp, encoding='utf-8') as f:
        txt = f.read()
    lines = [l for l in txt.splitlines() if l]
    ok = (len(lines) == 4 and '1ms tick' in lines[0] and '4/4 motors online' in lines[2]
          and 'partial line no newline' in lines[3])
    print(txt.rstrip())
    print(f'[selftest] 行数={len(lines)} (期望 4), 时间戳/切分/残行收尾: {"PASS" if ok else "FAIL"}')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='F407 串口日志采集器 (零依赖 Win32)')
    ap.add_argument('--port', default='auto', help='COM3 / auto(自动选第一个)')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--out', default=None, help='输出文件 (默认 logs/f407_serial_<时间>.log)')
    ap.add_argument('--duration', type=float, default=None, help='秒; 省略则 Ctrl-C 停止')
    ap.add_argument('--list', action='store_true', help='列出串口后退出')
    ap.add_argument('--probe', type=float, default=None, help='只探测: 打开并读 N 秒, 统计字节数')
    ap.add_argument('--no-stamp', action='store_true', help='不加时间戳')
    ap.add_argument('--selftest', action='store_true', help='自测落盘路径(不需要串口)')
    args = ap.parse_args()

    if args.selftest:
        return cmd_selftest()

    ports = list_ports()
    if args.list:
        print('可用串口:', ports if ports else '(无)')
        return 0

    port = args.port
    if port == 'auto':
        if not ports:
            print('[MOSS] 未发现任何串口 —— 检查 USB 转串口/驱动/线缆', file=sys.stderr)
            return 3
        port = ports[0]
        print(f'[MOSS] 自动选择 {port} (可用: {ports})')

    ser = Win32Serial(port, args.baud)
    try:
        ser.open()
    except OSError as e:
        print(f'[MOSS] 打开串口失败: {e}', file=sys.stderr)
        return 3
    print(f'[MOSS] {port} @ {args.baud} 8N1 已打开')

    if args.probe is not None:
        t0 = time.monotonic()
        total = 0
        while time.monotonic() - t0 < args.probe:
            total += len(ser.read())
            time.sleep(0.02)
        ser.close()
        print(f'[probe] {args.probe}s 内收到 {total} 字节 → {"有数据 ✓" if total else "无数据 (检查板子是否在打印/波特率/线序)"}')
        return 0 if total else 4

    out = args.out or os.path.join('logs',
                                   f'f407_serial_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')
    lg = Logger(out, stamp=not args.no_stamp)
    print(f'[MOSS] 落盘: {out}   (Ctrl-C 停止)')
    t0 = time.monotonic()
    try:
        while True:
            if args.duration and (time.monotonic() - t0) >= args.duration:
                break
            data = ser.read()
            if data:
                lg.feed(data)
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        print('\n[MOSS] 手动停止。')
    finally:
        ser.close()
        lg.close()
    print(f'[MOSS] 结束: {lg.nlines} 行 / {lg.nbytes} 字节 → {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
