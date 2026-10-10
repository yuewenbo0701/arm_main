# -*- coding: utf-8 -*-
"""
MPC_Z_P 开关工具 (按 a 关 Z 位置环 / 按 b 恢复)
====================================================================

用途:
    联调对比 Z 轴"位置定高"与"不定高(垂速锁零)"的手感差异。

    按 a : MPC_Z_P -> 0.0
           Z 位置环 P 增益归零, 位置误差不再把飞机拉回设定高度;
           注意速度环仍在闭环锁垂速 -> 推走后停在哪算哪, 效果等同
           alt_hold_toggle.py 的掩码 3548 + vz=0。
    按 b : MPC_Z_P 恢复到启动时读到的原值 (读不到则兜底 1.0)。
    按 q : 退出; 若当前处于"关"状态, 退出前自动恢复原值。

原理:
    MAVLink PARAM_SET (MAV_PARAM_TYPE_REAL32), 发送后等待 PX4 回显
    PARAM_VALUE 做读回验证, 验证失败自动重试 MAX_RETRY 次。

重要提醒:
    1. 这是全局参数, 影响所有模式 (Position / Altitude / Hold / RTL /
       自动航线), 不只是 Offboard。关掉后 failsafe 的 Z 刚度也是 0。
    2. PX4 会把 PARAM_SET 的值自动写进 flash。若脚本异常退出 (断电 /
       崩溃) 时正处于"关"状态, 下次上电 MPC_Z_P 仍是 0 —— 需重跑本
       脚本按 b 恢复, 或在 QGC 里改回。脚本启动时若发现值已为 0
       会醒目提示 (大概率是上次没恢复)。
    3. 仅用于地面联调 / 低空低速对比测试。

依赖: pip install pymavlink
运行: python z_pos_loop_toggle.py
"""

import sys
import time
import threading
from pymavlink import mavutil

# ========== 参数区 ==========
MAVLINK_ADDR = '/dev/ttyUSB0'   # Pixhawk USB 直连
MAVLINK_BAUD = 921600

PARAM_NAME        = 'MPC_Z_P'
OFF_VALUE         = 0.0
FALLBACK_ON_VALUE = 1.0         # 启动时读不到原值, 恢复时用此兜底 (PX4 默认 1.0)

VERIFY_TIMEOUT  = 2.0           # PARAM_VALUE 回显等待超时 (秒)
MAX_RETRY       = 3             # 设置失败重试次数
REREAD_INTERVAL = 3.0           # 空闲时周期性读回当前值, 让显示反映飞控真值

# ========== 全局状态 ==========
running = True
lock    = threading.Lock()
pending = None                  # None / 'off' / 'on' —— 键盘线程置位, 主循环执行


# ========== 键盘监听线程: a 关 / b 开 / q 退出 ==========
def keyboard_listener():
    global running, pending

    restore_termios = None
    try:
        import msvcrt                                  # Windows
        def read_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:                                # Linux / macOS
        import tty, termios, select
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        def restore_termios():                         # 退出时恢复终端, 否则 shell 会乱
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        def read_key():
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None

    print("\n[键盘] a=关Z位置环(MPC_Z_P=0)  b=恢复原值  q=退出(退出时自动恢复)")
    try:
        while running:
            try:
                key = read_key()
            except Exception:
                break
            if key in ('a', 'b'):
                with lock:
                    pending = 'off' if key == 'a' else 'on'
            elif key == 'q':
                running = False
                break
            time.sleep(0.05)
    finally:
        if restore_termios:
            restore_termios()


# ========== 参数读写 (PARAM_VALUE 回显验证) ==========
def _match_param_value(master, deadline):
    """在 deadline 前等一条名为 PARAM_NAME 的 PARAM_VALUE, 返回其值; 超时返回 None"""
    while True:
        remain = deadline - time.time()
        if remain <= 0:
            return None
        msg = master.recv_match(type='PARAM_VALUE', blocking=True,
                                timeout=max(0.05, remain))
        if msg is None:
            return None
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('ascii', errors='ignore')
        if name.strip('\x00') == PARAM_NAME:
            return msg.param_value


def get_param(master):
    """PARAM_REQUEST_READ 读当前值; 超时返回 None"""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        PARAM_NAME.encode('ascii'), -1
    )
    return _match_param_value(master, time.time() + VERIFY_TIMEOUT)


def set_param(master, value):
    """PARAM_SET + 回显验证, 失败重试 MAX_RETRY 次。返回 True/False"""
    for attempt in range(1, MAX_RETRY + 1):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            PARAM_NAME.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        echo = _match_param_value(master, time.time() + VERIFY_TIMEOUT)
        if echo is not None and abs(echo - value) < 1e-4:
            return True
        print(f"\n[重试 {attempt}/{MAX_RETRY}] 未收到正确回显, 重发 PARAM_SET ...")
    return False


# ========== 主流程 ==========
def main():
    global running

    print(f"连接飞控 {MAVLINK_ADDR} @ {MAVLINK_BAUD} ...")
    master = mavutil.mavlink_connection(MAVLINK_ADDR, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print(f"已连接: sysid={master.target_system} compid={master.target_component}")

    # ---- 启动时读原值: 恢复目标 = 读到的值, 读不到用兜底 ----
    on_value = FALLBACK_ON_VALUE
    cur = get_param(master)
    if cur is None:
        print(f"[警告] 读不到 {PARAM_NAME}, 恢复值用兜底 {FALLBACK_ON_VALUE}")
    else:
        print(f"当前 {PARAM_NAME} = {cur:.3f}")
        if abs(cur) < 1e-6:
            print("!! 警告: 当前值已是 0 —— 很可能上次异常退出没恢复, 按 b 可恢复正常 !!")
        else:
            on_value = cur

    is_off = False

    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    last_reread = time.time()

    try:
        while running:
            # ---- 取键盘请求 (置位即取走) ----
            global pending
            with lock:
                req = pending
                pending = None

            if req == 'off':
                if set_param(master, OFF_VALUE):
                    is_off = True
                    print(f">>> [a] {PARAM_NAME} = {OFF_VALUE:.3f} 已生效 "
                          f"—— Z 位置环已关 (不定高; 垂速环仍在锁零)")
                else:
                    print(f">>> [a] 设置失败! 状态未变, 可再按一次 a")
                last_reread = time.time()

            elif req == 'on':
                if set_param(master, on_value):
                    is_off = False
                    print(f">>> [b] {PARAM_NAME} = {on_value:.3f} 已恢复 "
                          f"—— Z 位置环已开 (恢复定高)")
                else:
                    print(f">>> [b] 恢复失败! 请立即再按一次 b !!")
                last_reread = time.time()

            else:
                time.sleep(0.05)

            # ---- 空闲时周期读回, 显示飞控侧真值 (QGC 改动也能看到) ----
            if time.time() - last_reread > REREAD_INTERVAL:
                v = get_param(master)
                if v is not None:
                    state = "位置环关(不定高)" if abs(v) < 1e-6 else "位置环开"
                    print(f"\r[状态] {PARAM_NAME} = {v:.3f} | {state}   ", end="")
                    is_off = abs(v) < 1e-6
                last_reread = time.time()

    except KeyboardInterrupt:
        print("\n停止。")

    finally:
        running = False
        t.join(timeout=1.0)     # 让键盘线程恢复终端设置

        # ---- 退出保险: 处于"关"状态则自动恢复 ----
        if is_off:
            print(f"\n退出前自动恢复 {PARAM_NAME} 到 {on_value:.3f} ...")
            if set_param(master, on_value):
                print("已恢复。")
            else:
                print(f"!! 自动恢复失败! 请重跑本脚本按 b, 或在 QGC 把 "
                      f"{PARAM_NAME} 改回 {on_value:.3f} !!")
        master.close()


if __name__ == '__main__':
    main()
