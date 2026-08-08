#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
trae-ecc 最小可用上位机 v0.9
功能：心跳发送 + 上报解析 + 命令下发 + 急停
依赖：Python 3.10+, 标准库 (socket, struct, threading)
"""

import socket
import struct
import threading
import time
import sys
import os
from typing import Optional, Tuple

# ============================================================
# 协议定义 (与 contract.h 完全一致, 小端 PACKED)
# ============================================================

# 帧类型
FRAME_TYPE_REPORT    = 0x01
FRAME_TYPE_COMMAND   = 0x02
FRAME_TYPE_HEARTBEAT = 0x0F

# 控制模式
CTRL_MODE_POSITION = 0
CTRL_MODE_TORQUE   = 1
CTRL_MODE_MIXED    = 2

# 故障码
FAULT_CODES = {
    0x0001: "ESTOP",
    0x0002: "CAN1_TIMEOUT",
    0x0004: "CAN2_TIMEOUT",
    0x0008: "POSITION_JUMP",
    0x0010: "VELOCITY_LIMIT",
    0x0020: "TORQUE_LIMIT",
    0x0040: "VOLTAGE_LOW",
    0x0080: "WEIGHT_OPEN",
    0x0100: "COMM_LOST",
}

# 关节ID映射 (与 bsp_config.h / contract.h 完全一致)
JOINT_NAMES = {
    0x01: "左髋 (L-Hip)",
    0x02: "右髋 (R-Hip)",   # v1.6.3+fix2: CAN1=双髋控制(左/右), 非髋膝
    0x10: "手臂1 (Arm-1)",
    0x11: "手臂2 (Arm-2)",
    0x12: "手臂3 (Arm-3)",
    0x13: "手臂4 (Arm-4)",
}

# 网络配置 (与固件一致)
TARGET_IP   = "192.168.10.88"
TARGET_PORT = 5001
LOCAL_PORT  = 5001

# 帧大小 (与 contract.h 一致, v1.1 +ABO params)
FRAME_HEADER_SIZE   = 8
JOINT_STATUS_SIZE   = 17
JOINT_COMMAND_SIZE  = 22    # v1.1: ABO 6 字节字段 (abo_enable+assist_gain_q10+hpf_alpha_q16+bias_leak_q16) 新增, 16→22
REPORT_FRAME_SIZE   = FRAME_HEADER_SIZE + 6 * JOINT_STATUS_SIZE + 2   # 8+6*17+2 = 112
COMMAND_FRAME_SIZE  = FRAME_HEADER_SIZE + 6 * JOINT_COMMAND_SIZE + 2   # 8+6*22+2 = 142 (v1.1 变大)
HEARTBEAT_FRAME_SIZE = FRAME_HEADER_SIZE + 2 + 2                        # 12

# ============================================================
# CRC16 MODBUS (poly=0xA001, init=0xFFFF, refin=true, refout=true)
# 与固件 crc16() 函数完全一致
# ============================================================
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

# ============================================================
# 网卡选择 (避免 Docker/VMware 虚拟网卡抢流量)
# ============================================================
def list_network_interfaces() -> list:
    """枚举所有网卡并返回 [(name, ip), ...]"""
    import platform
    interfaces = []

    if platform.system() == "Windows":
        try:
            import ctypes
            import socket as _s
            # Windows: 使用 socket.gethostbyname_ex 获取本机 IP
            hostname = _s.gethostname()
            try:
                _, _, ips = _s.gethostbyname_ex(hostname)
                for ip in ips:
                    if ip.startswith("192.168.") or ip.startswith("10."):
                        interfaces.append((f"本地连接 ({ip})", ip))
            except:
                pass
            # 另一种方式: 遍历所有地址
            for family, _, _, _, sockaddr in _s.getaddrinfo(None, 0):
                if family == _s.AF_INET:
                    ip = sockaddr[0]
                    if not ip.startswith("127.") and ip not in [i[1] for i in interfaces]:
                        interfaces.append((f"网卡 ({ip})", ip))
        except Exception as e:
            print(f"[WARN] 枚举网卡失败: {e}")
    else:
        # Linux/macOS
        try:
            import subprocess
            result = subprocess.run(["ip", "addr"], capture_output=True, text=True)
            for line in result.stdout.split("\n"):
                if "inet " in line:
                    parts = line.strip().split()
                    ip = parts[1].split("/")[0]
                    if not ip.startswith("127."):
                        interfaces.append((f"iface ({ip})", ip))
        except:
            pass

    if not interfaces:
        interfaces.append(("默认 (0.0.0.0)", "0.0.0.0"))

    return interfaces

def select_network_interface() -> str:
    """让用户选择绑定的网卡IP"""
    interfaces = list_network_interfaces()

    print("\n========== 网卡选择 ==========")
    print("检测到以下可用网卡：")
    for i, (name, ip) in enumerate(interfaces):
        marker = "  ← 推荐" if ip.startswith("192.168.10.") else ""
        print(f"  [{i+1}] {name}{marker}")
    print(f"  [0] 手动输入 IP")

    while True:
        try:
            choice = input("\n请选择网卡序号 (0-{}): ".format(len(interfaces))).strip()
            if choice == "0":
                ip = input("请输入本机绑定 IP: ").strip()
                return ip
            idx = int(choice) - 1
            if 0 <= idx < len(interfaces):
                return interfaces[idx][1]
        except (ValueError, KeyboardInterrupt):
            print("输入无效，请重试")

# ============================================================
# 帧构造函数
# ============================================================

def build_heartbeat_frame(seq_num: int, data: int = 0) -> bytes:
    """构造心跳帧 (12字节)
    data=0: 正常心跳
    data=0xFFFF: 急停
    """
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF
    # FrameHeader: frame_type(1) + reserved(1) + seq_num(2) + timestamp(4) = 8
    header = struct.pack("<BBHI", FRAME_TYPE_HEARTBEAT, 0, seq_num & 0xFFFF, timestamp)
    data_field = struct.pack("<H", data & 0xFFFF)
    # CRC 覆盖 header + data_field (共 10 字节)
    crc = crc16_modbus(header + data_field)
    return header + data_field + struct.pack("<H", crc)

def build_command_frame(seq_num: int, commands: list) -> bytes:
    """构造命令帧 (142字节, v1.1 +ABO params)
    commands: list of dict, 每个 dict 含 joint_id, control_mode, syn_target, ...
    JointCommand_t = 22字节:
      joint_id(1) + mode(1) + syn_target(4) + max_vel(2) + max_acc(2) + trq_rate(2)
      + pid_idx(1) + kp(1) + kd(1) + abo_enable(1) + gain(2) + alpha(2) + leak(2)
    """
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF
    header = struct.pack("<BBHI", FRAME_TYPE_COMMAND, 0, seq_num & 0xFFFF, timestamp)
    body = b""
    for i in range(6):
        if i < len(commands):
            cmd = commands[i]
        else:
            cmd = {"joint_id": 0, "control_mode": 0, "syn_target": 0,
                   "max_velocity": 0, "max_acceleration": 0, "torque_rate_limit": 0,
                   "pid_set_index": 0, "kp": 0, "kd": 0,
                   "abo_enable": 1, "assist_gain_q10": 1024,
                   "hpf_alpha_q16": 655, "bias_leak_q16": 66}
        # JointCommand_t: 22字节 (小端, v1.1 with ABO fields)
        body += struct.pack("<BBiHHHBBBBHHH",
            cmd.get("joint_id", 0),
            cmd.get("control_mode", 0),
            cmd.get("syn_target", 0),
            cmd.get("max_velocity", 0),
            cmd.get("max_acceleration", 0),
            cmd.get("torque_rate_limit", 0),
            cmd.get("pid_set_index", 0),
            cmd.get("kp", 0),
            cmd.get("kd", 0),
            cmd.get("abo_enable", 1),
            cmd.get("assist_gain_q10", 1024),
            cmd.get("hpf_alpha_q16", 655),
            cmd.get("bias_leak_q16", 66),
        )
    crc = crc16_modbus(header + body)
    return header + body + struct.pack("<H", crc)

# ============================================================
# 帧解析函数
# ============================================================

def parse_report_frame(data: bytes) -> Optional[dict]:
    """解析上报帧 (118字节)"""
    if len(data) < REPORT_FRAME_SIZE:
        print(f"[WARN] 上报帧长度不足: {len(data)} < {REPORT_FRAME_SIZE}")
        return None

    # 校验 CRC
    received_crc = struct.unpack_from("<H", data, REPORT_FRAME_SIZE - 2)[0]
    calc_crc = crc16_modbus(data[:REPORT_FRAME_SIZE - 2])
    if received_crc != calc_crc:
        print(f"[WARN] CRC 错误: 收到 0x{received_crc:04X}, 计算 0x{calc_crc:04X}")
        return None

    # 解析帧头
    frame_type, reserved, seq_num, timestamp = struct.unpack_from("<BBHI", data, 0)
    if frame_type != FRAME_TYPE_REPORT:
        return None

    # 解析 6 个关节状态
    joints = []
    offset = FRAME_HEADER_SIZE
    for i in range(6):
        joint_id, position, velocity, torque, temperature, fault = struct.unpack_from(
            "<BiiiHH", data, offset
        )
        joints.append({
            "joint_id": joint_id,
            "position": position,        # mdeg
            "velocity": velocity,        # mdeg/s
            "torque": torque,            # mNm
            "temperature": temperature,  # 0.1°C
            "fault": fault,
        })
        offset += JOINT_STATUS_SIZE

    return {
        "seq_num": seq_num,
        "timestamp": timestamp,
        "joints": joints,
    }

def decode_faults(fault_code: int) -> list:
    """将故障码解码为字符串列表"""
    faults = []
    for bit, name in FAULT_CODES.items():
        if fault_code & bit:
            faults.append(name)
    return faults

# ============================================================
# 全局状态
# ============================================================

class AppState:
    def __init__(self):
        self.running = True
        self.seq_num = 0
        self.last_report = None
        self.last_report_time = 0
        self.report_count = 0
        self.heartbeat_count = 0
        self.command_count = 0

state = AppState()

# ============================================================
# 心跳线程 (200ms 周期)
# ============================================================

def heartbeat_thread(sock: socket.socket):
    """200ms 固定周期发送心跳帧"""
    print("[HEARTBEAT] 心跳线程启动 (200ms 周期)")
    while state.running:
        try:
            state.seq_num += 1
            frame = build_heartbeat_frame(state.seq_num, data=0)
            sock.sendto(frame, (TARGET_IP, TARGET_PORT))
            state.heartbeat_count += 1
        except Exception as e:
            print(f"[HEARTBEAT] 发送失败: {e}")
        time.sleep(0.2)  # 200ms
    print("[HEARTBEAT] 心跳线程停止")

# ============================================================
# 接收线程
# ============================================================

def receive_thread(sock: socket.socket):
    """接收并解析上报帧"""
    print("[RECEIVE] 接收线程启动")
    sock.settimeout(0.5)
    while state.running:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= REPORT_FRAME_SIZE:
                report = parse_report_frame(data)
                if report:
                    state.last_report = report
                    state.last_report_time = time.time()
                    state.report_count += 1
        except socket.timeout:
            continue
        except Exception as e:
            if state.running:
                print(f"[RECEIVE] 错误: {e}")
    print("[RECEIVE] 接收线程停止")

# ============================================================
# 显示线程 (300ms 刷新一次终端显示)
# ============================================================

def display_thread():
    """300ms 刷新终端显示"""
    print("[DISPLAY] 显示线程启动")
    last_display = 0
    while state.running:
        now = time.time()
        if now - last_display >= 0.3:
            last_display = now
            _clear_screen()
            _print_status()
        time.sleep(0.05)
    print("[DISPLAY] 显示线程停止")

def _clear_screen():
    os.system("cls" if os.name == "nt" else "clear")

def _print_status():
    print("=" * 70)
    print("  trae-ecc 最小上位机 v0.9")
    print(f"  目标: {TARGET_IP}:{TARGET_PORT} | 心跳: {state.heartbeat_count} | 上报: {state.report_count}")
    print("=" * 70)

    if state.last_report is None:
        print("\n  [等待上报帧...] 请检查网线连接和固件运行状态")
    else:
        report = state.last_report
        age = time.time() - state.last_report_time
        print(f"\n  上报序号: {report['seq_num']}  时间戳: {report['timestamp']}ms  ({age:.1f}s 前)")
        print("-" * 70)
        print(f"  {'关节':<16} {'位置(°)':>10} {'速度(°/s)':>10} {'力矩(Nm)':>10} {'温度(°C)':>8}  故障")
        print("-" * 70)

        for j in report["joints"]:
            jid = j["joint_id"]
            if jid == 0:
                continue
            name = JOINT_NAMES.get(jid, f"ID=0x{jid:02X}")
            pos_deg   = j["position"] / 1000.0
            vel_deg_s = j["velocity"] / 1000.0
            trq_nm    = j["torque"] / 1000.0
            temp_deg  = j["temperature"] / 10.0
            faults    = decode_faults(j["fault"])
            fault_str = ",".join(faults) if faults else "-"
            print(f"  {name:<16} {pos_deg:>10.2f} {vel_deg_s:>10.2f} {trq_nm:>10.3f} {temp_deg:>8.1f}  {fault_str}")

    # 全局故障
    if state.last_report:
        all_faults = set()
        for j in state.last_report["joints"]:
            all_faults.update(decode_faults(j["fault"]))
        if all_faults:
            print("-" * 70)
            print(f"  ⚠️  全局故障: {', '.join(sorted(all_faults))}")

    print("-" * 70)
    print("  命令: [space]急停  [1~6]选关节  [p]位置  [t]力矩  [q]退出")
    print("        例如: 输入 '1 p 30000' → 左髋转到 30° (位置模式)")
    print("             输入 '1 t 500' → 左髋 0.5Nm (力矩模式)")
    print("=" * 70)

# ============================================================
# 命令输入处理
# ============================================================

def send_estop(sock: socket.socket):
    """发送急停 (Heartbeat data=0xFFFF)"""
    state.seq_num += 1
    frame = build_heartbeat_frame(state.seq_num, data=0xFFFF)
    sock.sendto(frame, (TARGET_IP, TARGET_PORT))
    print("[CMD] 急停已发送")

def send_joint_command(sock: socket.socket, joint_idx: int, mode: int, value: int):
    """发送单关节命令
    joint_idx: 0~5 (对应腿0/腿1/臂0~3)
    mode: 0=位置, 1=力矩
    value: 位置=mdeg, 力矩=mNm
    """
    # 关节ID映射
    joint_ids = [0x01, 0x02, 0x10, 0x11, 0x12, 0x13]
    joint_id = joint_ids[joint_idx] if 0 <= joint_idx < 6 else joint_idx

    commands = [{"joint_id": 0, "control_mode": 0, "syn_target": 0,
                 "max_velocity": 0, "max_acceleration": 0, "torque_rate_limit": 0,
                 "pid_set_index": 0, "kp": 0, "kd": 0, "reserved": 0} for _ in range(6)]
    commands[joint_idx]["joint_id"] = joint_id
    commands[joint_idx]["control_mode"] = mode
    commands[joint_idx]["syn_target"] = value
    # 给一些默认限制
    commands[joint_idx]["max_velocity"] = 60000   # 60°/s
    commands[joint_idx]["max_acceleration"] = 10000

    state.seq_num += 1
    frame = build_command_frame(state.seq_num, commands)
    sock.sendto(frame, (TARGET_IP, TARGET_PORT))
    state.command_count += 1

    mode_str = "位置" if mode == 0 else "力矩"
    unit = "mdeg" if mode == 0 else "mNm"
    name = JOINT_NAMES.get(joint_id, f"ID=0x{joint_id:02X}")
    print(f"[CMD] {name}: {mode_str}={value}{unit}")

def command_input_thread(sock: socket.socket):
    """命令行输入处理线程"""
    print("\n[INPUT] 命令输入就绪，直接输入命令回车即可")
    print("        提示: 急停直接按 Enter 不输入内容也可触发")

    while state.running:
        try:
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            state.running = False
            break

        if not line:
            continue

        parts = line.split()

        # 急停
        if parts[0].lower() in ("space", "estop", "stop", "s"):
            send_estop(sock)
            continue

        # 退出
        if parts[0].lower() in ("q", "quit", "exit"):
            state.running = False
            break

        # 关节命令: <joint_num> <mode> <value>
        # joint_num: 1=左髋, 2=右髋, 3=手臂1, 4=手臂2, 5=手臂3, 6=手臂4
        if len(parts) >= 3:
            try:
                joint_num = int(parts[0])
                mode_str = parts[1].lower()
                value = int(parts[2])

                if 1 <= joint_num <= 6:
                    joint_idx = joint_num - 1
                    if mode_str in ("p", "pos", "position"):
                        send_joint_command(sock, joint_idx, CTRL_MODE_POSITION, value)
                    elif mode_str in ("t", "trq", "torque"):
                        send_joint_command(sock, joint_idx, CTRL_MODE_TORQUE, value)
                    else:
                        print(f"[INPUT] 未知模式: {mode_str}，使用 p 或 t")
                else:
                    print(f"[INPUT] 关节序号范围: 1~6")
            except ValueError:
                print(f"[INPUT] 命令格式错误: {line}")
        else:
            print(f"[INPUT] 命令太短，格式: <关节号> <模式> <数值>")

# ============================================================
# 主函数
# ============================================================

def main():
    print("=" * 70)
    print("  trae-ecc 最小可用上位机 v0.9")
    print("  功能: 心跳(200ms) + 上报解析 + 命令下发 + 急停")
    print("=" * 70)

    # 选择网卡
    local_ip = select_network_interface()
    print(f"\n[NET] 绑定本机 IP: {local_ip}:{LOCAL_PORT}")
    print(f"[NET] 目标固件 IP: {TARGET_IP}:{TARGET_PORT}")

    # 创建 UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((local_ip, LOCAL_PORT))
    except OSError as e:
        print(f"[ERROR] 绑定端口失败: {e}")
        print("        尝试绑定 0.0.0.0 ...")
        sock.bind(("0.0.0.0", LOCAL_PORT))

    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print("[NET] UDP Socket 创建成功")

    print("\n[INFO] 启动线程...")
    threads = [
        threading.Thread(target=heartbeat_thread, args=(sock,), daemon=True, name="heartbeat"),
        threading.Thread(target=receive_thread,   args=(sock,), daemon=True, name="receive"),
        threading.Thread(target=display_thread,   daemon=True, name="display"),
        threading.Thread(target=command_input_thread, args=(sock,), daemon=True, name="input"),
    ]
    for t in threads:
        t.start()

    print("[INFO] 所有线程已启动，等待上报帧...\n")
    print("  提示:")
    print("    • 如果长时间无上报，请检查: ping 192.168.10.88")
    print("    • 急停: 输入 space 或 s 回车")
    print("    • 退出: 输入 q 回车\n")

    # 主线程等待退出
    try:
        while state.running:
            time.sleep(0.2)
            # 检测上报超时 (600ms = 3个心跳周期)
            if state.last_report_time > 0:
                age = time.time() - state.last_report_time
                if age > 0.6:
                    pass  # 由终端显示提示
    except KeyboardInterrupt:
        print("\n[INFO] 收到 Ctrl+C，退出中...")
        state.running = False

    # 清理
    print("[INFO] 等待线程结束...")
    time.sleep(0.5)
    sock.close()
    print("[INFO] 已退出")

if __name__ == "__main__":
    main()
