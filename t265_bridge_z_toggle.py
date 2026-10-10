# -*- coding: utf-8 -*-
"""
T265 -> PX4 桥接 + Z 轴控制参数开关 + 地面自检 (三合一版, 三参数)
====================================================================

由 rpi/t265_px4_bridge.py 与 z_pos_loop_toggle.py 合并而来。

功能 1 (视觉桥接, 与 t265_px4_bridge.py 相同):
    读取 T265 位姿, 转换到 NED 系 (X前/Y右/Z下), 降频到 30Hz
    通过 VISION_POSITION_ESTIMATE 发给 Pixhawk, 供 EKF2 融合定点。
    相机安装朝向: 水平面内偏转 90°, 重映射关系与原版一致。

功能 2 (三参数开关):
    按 a : 三个参数同时进入"测试态":
             MPC_Z_P = 0          位置环不回拉: 被推后不再回原高度
             MPC_Z_VEL_I_ACC = 0  速度环无积分: 稳态误差无人补偿,
                                  悬停油门模型偏差/电池压降/机械臂载重
                                  变化都会变成持续缓慢爬升或下沉
             MPC_Z_VEL_P_ACC = 2  速度环比例减半(默认4): 同样垂速误差
                                  纠偏加速度减半 -> 刹车变软, 被推后
                                  滑得更远, 抗风变差
           三者叠加: 既不回原高度, 又不锁稳态, 刹车还肉 —— Z 通道
           明显变"松"。PX4 的 Z 速度环 D 项默认就是 0, 测试态下
           速度环只剩 P=2 + 悬停前馈。
           !! 出现持续快速/加速漂移立即按 b !!
    按 b : 三个参数都恢复到启动时各自读到的原值 (读不到用兜底值)。
    按 t : 地面自检 (见功能 3)
    按 q : 退出; 退出时检测到哪个参数处于测试值就自动恢复哪个

    PX4 控制结构 (官方 Controller Diagrams 文档):
      位置环 = 纯 P (MPC_Z_P), 输出速度设定;
      速度环 = PID (MPC_Z_VEL_P_ACC / MPC_Z_VEL_I_ACC / MPC_Z_VEL_D_ACC),
               输出加速度(推力)设定, 积分项带 anti-reset windup。

功能 3 (地面自检, 9 步):
    读 x3 -> 三个都设为测试值(回显验证) -> 三个都恢复(回显验证),
    逐步打印 OK/FAIL, 最后 PASS/FAIL。
      - 台上自检 (不开 T265/键盘):  python t265_bridge_z_toggle.py --selftest
      - 运行中随时按 t: 异步执行, 视觉流和 TIMESYNC 不中断, 天上也能用。
    恢复目标是"测试开始时各自读到的值", 不会篡改你当前的开关状态。

    验证方式说明 (实测教训):
    自检用 PARAM_SET 的回显(PARAM_VALUE)做验证, 不依赖 PARAM_REQUEST_READ
    的响应。原因: MAVLink 协议规定 SET 后飞控必须广播已提交的当前值, 这个
    回显本身就是读回验证 (QGC 同理); 而实测发现 PX4 对 READ 请求的响应在
    链路繁忙时会延迟数秒甚至丢弃 (SET 回显却始终毫秒级), 用 READ 做验证
    只会产生误报。步骤 1-3 保留 READ, 仅用于确认链路并学习原值。

重要提醒:
    1. 三个都是全局参数, 影响所有模式 (Position/Hold/RTL/自动航线),
       不只是 Offboard; 关掉后 failsafe 的 Z 通道也会变软。
       仅用于地面联调 / 低空低速对比测试。
    2. 测试态下扰动恢复距离明显变长 (刹车软+无积分): 先轻推试刹车
       距离, 有数了再做大动作; 下沉漂移时离地至少 1.5m, 保持手可
       接管, 异常立即按 b。
    3. PX4 会把 PARAM_SET 自动写进 flash。异常退出时若参数处于测试值,
       下次上电仍是测试值 —— 重跑本脚本按 b, 或 QGC 改回。启动时
       若发现任一参数已处于测试值会醒目提示。
    4. 参数操作全部走异步队列 + PARAM_VALUE 回显验证 + 重试,
       不会阻塞 30Hz 视觉流和 TIMESYNC 应答。

依赖: pip install pyrealsense2 pymavlink numpy
运行: python t265_bridge_z_toggle.py             (桥接 + 开关)
      python t265_bridge_z_toggle.py --selftest  (纯自检, 不开相机)
"""

import sys
import time
import math
import threading
from collections import deque

import numpy as np
from pymavlink import mavutil

# ========== 参数区 ==========
MAVLINK_ADDR   = '/dev/ttyUSB0'   # Pixhawk USB 直连
MAVLINK_BAUD   = 921600
SEND_RATE_HZ   = 30.0             # 视觉定位发送频率 (T265 输出 200Hz, 降频发给飞控)
MIN_CONFIDENCE = 2                # tracker_confidence: 0失败 1低 2中 3高

# 开关管理的参数清单:
#   name     = PX4 参数名
#   off      = 按 a 时的"测试态"目标值 (注意 Z_VP 是改小不是置0)
#   fallback = 启动读不到原值时的恢复兜底 (PX4 默认值)
#   short    = 状态行缩写; note_off/note_on = 按键成功后的提示语
PARAMS = [
    {'name': 'MPC_Z_P',         'off': 0.0, 'fallback': 1.0, 'short': 'Z_P',
     'note_off': '—— Z位置环已关 (被推后不再回原高度)',
     'note_on':  '—— Z位置环已开 (恢复回原高度)'},
    {'name': 'MPC_Z_VEL_I_ACC', 'off': 0.0, 'fallback': 2.0, 'short': 'Z_I',
     'note_off': '—— Z速度环I项已关 (无稳态补偿, 悬停可能缓慢漂移!)',
     'note_on':  '—— Z速度环I项已恢复'},
    {'name': 'MPC_Z_VEL_P_ACC', 'off': 2.0, 'fallback': 4.0, 'short': 'Z_VP',
     'note_off': '—— Z速度环P减半 (刹车变软, 被推后滑得更远!)',
     'note_on':  '—— Z速度环P已恢复 (刹车复原)'},
]
TRACKED     = [p['name'] for p in PARAMS]
OFF_TARGET  = {p['name']: p['off'] for p in PARAMS}
FALLBACK_ON = {p['name']: p['fallback'] for p in PARAMS}
NOTES       = {(p['name'], k): p[k] for p in PARAMS for k in ('note_off', 'note_on')}

VERIFY_TIMEOUT  = 2.0             # SET 回显等待超时 (秒); SET 回显实测毫秒级
READ_TIMEOUT    = 4.0             # READ 响应等待超时 (秒); READ 响应实测可能数秒
MAX_RETRY       = 3               # 每个参数操作的重试次数
REREAD_INTERVAL = 3.0             # 空闲时周期性读回参数, 让显示反映飞控真值

# ========== 全局状态 ==========
running = True
lock    = threading.Lock()
pending = None                    # None / 'off' / 'on' / 'test' —— 键盘置位, 主循环消费

# 参数操作状态机 (仅主线程读写, 无需加锁); 值都按参数名分槽存放
P = {
    'cur':         {n: None for n in TRACKED},   # 最近一次 PARAM_VALUE 读到的真值
    'on_value':    dict(FALLBACK_ON),            # 恢复目标 (启动时学习)
    'on_fallback': {n: True for n in TRACKED},   # on_value 仍是兜底值, 读到真值后更新
    'current_op':  None,                         # 正在等待回显的操作
    'st_results':  [],                           # 自检各步骤 True/False
    'st_ctx':      {},                           # 自检上下文: {'start': {name: value}}
    'st_total':    0,
    'st_running':  False,
}
op_queue = deque()                # 待执行参数操作队列

def mod_count():
    """当前确认处于"测试值"的受管参数个数 (读不到的不计入)"""
    return sum(1 for n in TRACKED
               if P['cur'][n] is not None
               and abs(P['cur'][n] - OFF_TARGET[n]) < 1e-4)

def fmt(v):
    return f"{v:.3f}" if v is not None else "?"


# ========== 四元数工具 ==========
def quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quat_inverse(q):
    x, y, z, w = q
    norm2 = x*x + y*y + z*z + w*w
    return np.array([-x, -y, -z, w]) / norm2

# ========== T265 坐标系 -> NED 重映射 ==========
def remap_pos(p_t265):
    x, y, z = p_t265
    # 基础映射旧结果为 (-z, x, -y)，叠加90°旋转后:
    return np.array([-x, -z, -y])

def remap_quat(q_t265):
    x, y, z, w = q_t265
    return np.array([-z, x, -y, w])

# ========== 四元数 -> 欧拉角（ZYX，输出单位: 弧度） ==========
def quat_to_euler_rad(q):
    x, y, z, w = q
    pitch = -math.asin(np.clip(2.0 * (x * z - w * y), -1, 1))
    roll  = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    yaw   = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)
    return -pitch, roll, yaw


# ========== 键盘监听线程: a 测试态 / b 全恢复 / t 自检 / q 退出 ==========
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

    print("\n[键盘] a=测试态(Z_P=0,Z_I=0,Z_VP=2)  b=全部恢复原值  "
          "t=地面自检  q=退出(退出时自动恢复处于测试值的参数)")
    try:
        while running:
            try:
                key = read_key()
            except Exception:
                break
            if key in ('a', 'b', 't'):
                with lock:
                    pending = {'a': 'off', 'b': 'on', 't': 'test'}[key]
            elif key == 'q':
                running = False
                break
            time.sleep(0.05)
    finally:
        if restore_termios:
            restore_termios()


# ========== 阻塞式参数读写 (仅用于启动读取 / --selftest / 退出恢复) ==========
def _match_param_value(master, want_name, deadline):
    """在 deadline 前等一条名为 want_name 的 PARAM_VALUE, 返回其值; 超时返回 None"""
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
        if name.strip('\x00') == want_name:
            return msg.param_value


def get_param_blocking(master, name):
    """阻塞读一次 (调用方自行重试); 超时 READ_TIMEOUT, 读响应实测可能数秒"""
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        name.encode('ascii'), -1
    )
    return _match_param_value(master, name, time.time() + READ_TIMEOUT)


def set_param_blocking(master, name, value):
    for _ in range(MAX_RETRY):
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode('ascii'), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        echo = _match_param_value(master, name, time.time() + VERIFY_TIMEOUT)
        if echo is not None and abs(echo - value) < 1e-4:
            return True
    return False


def run_blocking_selftest(master):
    """--selftest 模式: 不开 T265/键盘, 阻塞式自检并打印每一步。
    9 步: 读 x3 -> 都设为测试值(回显验证) -> 都恢复(回显验证)。
    验证走 SET 回显: 协议规定 SET 后飞控必须广播已提交值, 回显即读回;
    不依赖实测不可靠的 PARAM_REQUEST_READ 响应 (仅前三步用它确认链路)。"""
    total = 3 * len(PARAMS)
    print(f"[自检] 阻塞模式: 读x{len(PARAMS)} -> 设测试值(回显) -> 恢复(回显)")
    results = []
    start = {}

    # ---- 步骤 1..3: 逐个读当前值 (各带重试) ----
    for i, p_ in enumerate(PARAMS):
        name = p_['name']
        v = None
        for _ in range(MAX_RETRY):
            v = get_param_blocking(master, name)
            if v is not None:
                break
        ok = v is not None
        if ok:
            start[name] = v
        print("[自检] %d/%d 读 %s ... " % (i + 1, total, name) +
              (f"OK ({v:.3f})" if ok else "FAIL"))
        results.append(ok)

    if all(results):
        # ---- 步骤 4..6: 都设为测试值, 回显验证 ----
        for i, p_ in enumerate(PARAMS):
            name = p_['name']
            ok = set_param_blocking(master, name, p_['off'])
            print("[自检] %d/%d 设 %s=%.3f (回显验证) ... %s" %
                  (len(PARAMS) + i + 1, total, name, p_['off'],
                   'OK' if ok else 'FAIL'))
            results.append(ok)

        # ---- 步骤 7..9: 都恢复原值, 回显验证 ----
        for i, p_ in enumerate(PARAMS):
            name = p_['name']
            ok = set_param_blocking(master, name, start[name])
            print("[自检] %d/%d 恢复 %s=%.3f (回显验证) ... %s" %
                  (2 * len(PARAMS) + i + 1, total, name, start[name],
                   'OK' if ok else 'FAIL'))
            results.append(ok)
    else:
        results += [False] * (2 * len(PARAMS))

    n = sum(results)
    restored = " ".join(f"{p_['short']}={fmt(start.get(p_['name']))}" for p_ in PARAMS)
    if n == total:
        print(f"[自检] 结果: PASS ({n}/{total}) —— 链路正常, 参数已恢复 {restored}")
    else:
        print(f"[自检] 结果: FAIL ({n}/{total}) —— 检查串口/波特率/固件版本")
    return n == total


# ========== 异步参数操作 (正常运行时使用, 不阻塞视觉流) ==========
def _send_op(master, op):
    if op['kind'] == 'set':
        master.mav.param_set_send(
            master.target_system, master.target_component,
            op['name'].encode('ascii'), float(op['value']),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    else:  # read
        master.mav.param_request_read_send(
            master.target_system, master.target_component,
            op['name'].encode('ascii'), -1
        )


def _complete_op(ok, echo):
    """当前操作结束: 按 tag 打印结果; 自检步骤累计, 齐了就出总结"""
    op = P['current_op']
    P['current_op'] = None
    if op is None:
        return
    tag = op['tag']

    if tag == 'toggle':
        if ok:
            key = 'note_off' if op.get('direction') == 'off' else 'note_on'
            note = NOTES.get((op['name'], key), '')
            print(f"\n>>> {op['label']} 成功: {op['name']} = {echo:.3f} {note}")
        else:
            print(f"\n>>> {op['label']} 失败! 状态未变, 可再按一次")

    elif tag == 'selftest':
        P['st_results'].append(bool(ok))
        if ok and op.get('store_start'):
            P['st_ctx'].setdefault('start', {})[op['name']] = echo
        tail = f" ({echo:.3f})" if (ok and echo is not None) else \
               (f" (读到 {echo})" if echo is not None else "")
        print(f"\n[自检] {op['label']} ... {'OK' if ok else 'FAIL'}{tail}")
        if len(P['st_results']) == P['st_total']:
            n = sum(P['st_results'])
            if n == P['st_total']:
                start = P['st_ctx'].get('start', {})
                restored = " ".join(f"{p_['short']}={fmt(start.get(p_['name']))}"
                                    for p_ in PARAMS)
                print(f"[自检] 结果: PASS ({n}/{P['st_total']}) —— 链路正常, "
                      f"参数已恢复 {restored}")
            else:
                print(f"[自检] 结果: FAIL ({n}/{P['st_total']}) —— "
                      f"检查串口/波特率/固件版本")
            P['st_running'] = False

    # tag == 'refresh': 静默, 只更新 P['cur'] 供状态行显示


def on_param_value(name, v):
    """drain 到一条受管参数的 PARAM_VALUE: 更新真值; 若在等回显则判定当前操作"""
    P['cur'][name] = v
    # 用"非测试值"的读数学习恢复目标 (避免把测试值学成原值)
    if P['on_fallback'][name] and abs(v - OFF_TARGET[name]) > 1e-4:
        P['on_value'][name] = v
        P['on_fallback'][name] = False

    op = P['current_op']
    if op is None or name != op['name']:
        return
    if op['kind'] == 'read':
        if op['expect'] is None or abs(v - op['expect']) < 1e-4:
            _complete_op(True, v)
    else:  # set: 回显值必须等于目标值; 不等于则继续等到超时再重试
        if abs(v - op['expect']) < 1e-4:
            _complete_op(True, v)


def process_param_ops(master):
    """主循环每次迭代调用: 空闲则从队列取一个操作发出; 超时则重试/判失败"""
    now = time.time()
    op = P['current_op']

    if op is None:
        while op_queue:
            op = op_queue.popleft()
            # 自检"恢复原值"步在此解析目标值: 优先测试开始时的读数,
            # 兜底当前已知真值, 再兜底 on_value, 保证一定能恢复
            if op['kind'] == 'set' and op['value'] is None:
                start = P['st_ctx'].get('start', {}).get(op['name'])
                if start is None:
                    start = P['cur'][op['name']]
                if start is None:
                    start = P['on_value'][op['name']]
                op['value'] = start
                op['expect'] = start
            timeout = READ_TIMEOUT if op['kind'] == 'read' else VERIFY_TIMEOUT
            op['attempt'] = 1
            _send_op(master, op)
            op['deadline'] = now + timeout
            P['current_op'] = op
            return
        return

    if now > op['deadline']:
        timeout = READ_TIMEOUT if op['kind'] == 'read' else VERIFY_TIMEOUT
        if op['attempt'] < MAX_RETRY:
            op['attempt'] += 1
            _send_op(master, op)
            op['deadline'] = now + timeout
            print(f"\n[重试 {op['attempt']}/{MAX_RETRY}] {op['label']}")
        else:
            _complete_op(False, None)


def enqueue_toggle(turn_off):
    """按 a/b: 三个参数依次入队 (逐个回显验证); a 进测试值, b 回原值"""
    for p_ in PARAMS:
        name = p_['name']
        target = p_['off'] if turn_off else P['on_value'][name]
        op_queue.append({
            'kind': 'set', 'name': name, 'value': target, 'expect': target,
            'tag': 'toggle', 'direction': 'off' if turn_off else 'on',
            'label': ('[a] ' if turn_off else '[b] ') +
                     ('测试值 ' if turn_off else '恢复 ') + name,
        })


def enqueue_selftest():
    """按 t: 异步自检 —— 9 步依次入队, 期间视觉/TIMESYNC 不中断。
    验证走 SET 回显 (协议规定 SET 后飞控必须广播已提交值, 回显即读回);
    不再依赖实测会延迟/丢弃的 PARAM_REQUEST_READ 响应。"""
    if P['st_running']:
        print("\n[自检] 已在进行中, 忽略")
        return
    P['st_running'] = True
    P['st_results'] = []
    P['st_ctx'] = {'start': {n: P['cur'][n] for n in TRACKED}}

    total = 3 * len(PARAMS)
    ops = []
    step = 0
    for p_ in PARAMS:                       # 读 x3
        step += 1
        ops.append({'kind': 'read', 'name': p_['name'], 'expect': None,
                    'store_start': True, 'tag': 'selftest',
                    'label': '%d/%d 读 %s' % (step, total, p_['name'])})
    for p_ in PARAMS:                       # 设测试值 x3
        step += 1
        ops.append({'kind': 'set', 'name': p_['name'], 'value': p_['off'],
                    'expect': p_['off'], 'tag': 'selftest',
                    'label': '%d/%d 设 %s=%.3f (回显验证)' %
                             (step, total, p_['name'], p_['off'])})
    for p_ in PARAMS:                       # 恢复 x3 (值出队时解析)
        step += 1
        ops.append({'kind': 'set', 'name': p_['name'], 'value': None,
                    'expect': 'start', 'tag': 'selftest',
                    'label': '%d/%d 恢复 %s (回显验证)' % (step, total, p_['name'])})

    P['st_total'] = len(ops)
    op_queue.extend(ops)
    print(f"\n[自检] 开始 (异步, 视觉流不中断): 读x{len(PARAMS)} -> "
          f"设测试值(回显) -> 恢复(回显)")


def drain_messages(master):
    """非阻塞排空接收缓冲: 应答 TIMESYNC; 捕获受管参数的 PARAM_VALUE"""
    while True:
        msg = master.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == 'TIMESYNC':
            master.mav.timesync_send(int(time.time_ns()), msg.ts1)
        elif mtype == 'PARAM_VALUE':
            name = msg.param_id
            if isinstance(name, bytes):
                name = name.decode('ascii', errors='ignore')
            name = name.strip('\x00')
            if name in TRACKED:
                on_param_value(name, msg.param_value)


def param_state_str():
    parts = []
    known = 0
    for p_ in PARAMS:
        v = P['cur'][p_['name']]
        parts.append(f"{p_['short']}={fmt(v)}")
        if v is not None:
            known += 1
    if known == 0:
        state = "?"
    elif mod_count() == len(PARAMS):
        state = "测试态(不回高+无I+软P)"
    elif mod_count() == 0:
        state = "原值态"
    else:
        state = "部分改(%d/%d)" % (mod_count(), len(PARAMS))
    return " ".join(parts) + f" [{state}]"


# ========== 主流程 ==========
def main():
    global running, pending

    # ---- 连接飞控 ----
    print(f"连接飞控 {MAVLINK_ADDR} @ {MAVLINK_BAUD} ...")
    master = mavutil.mavlink_connection(MAVLINK_ADDR, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    print(f"已连接: sysid={master.target_system} compid={master.target_component}")

    # ---- --selftest: 纯自检, 不开 T265/键盘, 跑完就退出 ----
    if '--selftest' in sys.argv:
        try:
            ok = run_blocking_selftest(master)
        finally:
            master.close()
        sys.exit(0 if ok else 1)

    # ---- 启动时逐个读原值: 恢复目标 = 读到的值, 读不到用兜底 ----
    for p_ in PARAMS:
        name = p_['name']
        cur = get_param_blocking(master, name)
        if cur is None:
            print(f"[警告] 读不到 {name}, 恢复值先用兜底 {FALLBACK_ON[name]}, "
                  f"稍后读到真值会自动更新")
        else:
            P['cur'][name] = cur
            print(f"当前 {name} = {cur:.3f}")
            if abs(cur - p_['off']) < 1e-4:
                print(f"!! 警告: {name} 已处于测试值 {p_['off']:.3f} —— 很可能"
                      f"上次异常退出没恢复, 按 b 可恢复正常 !!")
            else:
                P['on_value'][name] = cur
                P['on_fallback'][name] = False

    # ---- 启动 T265 (失败则降级为纯参数开关模式, 桥接功能关闭) ----
    pipe = None
    try:
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        pipe.start(cfg)
        print("T265 已启动，等待位姿数据...")
    except Exception as e:
        print(f"[警告] T265 启动失败 ({e}), 降级为纯参数开关模式 (无视觉桥接)")
        pipe = None

    # ---- 启动键盘监听 ----
    t = threading.Thread(target=keyboard_listener, daemon=True)
    t.start()

    print(f"[发送] 视觉定位 {SEND_RATE_HZ:.0f}Hz" if pipe else "[模式] 纯参数开关")

    # ---- 定时器 ----
    last_vision_send = 0.0
    last_refresh     = time.time()
    last_status      = 0.0

    send_count = 0
    drop_count = 0

    initial_pos   = None
    inv_init_quat = None

    try:
        while running:
            # ---- 取帧 / 纯参数模式走 sleep ----
            frames = None
            if pipe is not None:
                # 100ms 超时: 即使相机帧没来, 循环也能准时走,
                # 保证 TIMESYNC 应答和参数操作不停摆
                try:
                    frames = pipe.wait_for_frames(100)
                except RuntimeError:
                    frames = None
            else:
                time.sleep(0.05)

            # ---- 应答 TIMESYNC + 捕获 PARAM_VALUE ----
            drain_messages(master)

            # ---- 消费键盘请求 -> 入队参数操作 ----
            with lock:
                req = pending
                pending = None
            if req == 'off':
                enqueue_toggle(turn_off=True)
            elif req == 'on':
                enqueue_toggle(turn_off=False)
            elif req == 'test':
                enqueue_selftest()

            # ---- 推进参数操作状态机 (发/重试/判超时) ----
            process_param_ops(master)

            now = time.time()

            # ---- 空闲时周期读回三个参数, 让状态行反映飞控真值 ----
            if now - last_refresh > REREAD_INTERVAL:
                last_refresh = now
                if P['current_op'] is None and not op_queue:
                    for name in TRACKED:
                        op_queue.append({'kind': 'read', 'name': name,
                                         'expect': None,
                                         'tag': 'refresh', 'label': '刷新'})

            # ---- 纯参数模式: 定期刷状态行; 视觉模式的状态行走下面 30Hz ----
            if pipe is None:
                if now - last_status > 2.0:
                    last_status = now
                    print(f"\r[状态] {param_state_str()}   ", end="")
                continue

            if frames is None:
                continue
            pose_frame = frames.get_pose_frame()
            if not pose_frame:
                continue
            pose = pose_frame.get_pose_data()

            # ---- 视觉定位降频 ----
            if now - last_vision_send < 1.0 / SEND_RATE_HZ:
                continue
            last_vision_send = now

            # ---- 跟踪质量检查：丢失跟踪时不发，避免毒化 EKF ----
            if pose.tracker_confidence < MIN_CONFIDENCE:
                drop_count += 1
                print(f"\r[警告] 跟踪置信度低({pose.tracker_confidence})，暂停发送视觉数据，已丢 {drop_count} 帧",
                      end="")
                continue

            # ---- 坐标转换 ----
            pos = remap_pos(np.array([
                pose.translation.x,
                pose.translation.y,
                pose.translation.z
            ]))
            quat = remap_quat(np.array([
                pose.rotation.x,
                pose.rotation.y,
                pose.rotation.z,
                pose.rotation.w
            ]))

            if initial_pos is None:
                initial_pos = pos.copy()
                inv_init_quat = quat_inverse(quat.copy())
                rel_euler = quat_to_euler_rad(quat)
                print("基准已设定")
                print(f"初始位置: X前={initial_pos[0]:.3f} "
                      f"Y右={initial_pos[1]:.3f} Z下={initial_pos[2]:.3f} m")
                print(f"初始姿态: Roll={rel_euler[0]:.1f} "
                      f"Pitch={rel_euler[1]:.1f} Yaw={rel_euler[2]:.1f}°")
                continue

            quat = quat_mul(inv_init_quat, quat)
            roll, pitch, yaw = quat_to_euler_rad(quat)

            # ---- 发送 VISION_POSITION_ESTIMATE ----
            # 协方差第一元素置 NaN -> PX4 使用 EKF2_EVP_NOISE / EKF2_EVA_NOISE 默认噪声
            cov = [float('nan')] * 21
            master.mav.vision_position_estimate_send(
                int(now * 1e6),                  # UNIX 时间戳(us)，靠 TIMESYNC 对齐
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(roll), float(pitch), float(yaw),
                cov
            )
            send_count += 1

            print(
                f"\r已发送 {send_count} | "
                f"X前={pos[0]:+.3f} Y右={pos[1]:+.3f} Z下={pos[2]:+.3f} m | "
                f"R={math.degrees(roll):+.1f} P={math.degrees(pitch):+.1f} "
                f"Y={math.degrees(yaw):+.1f}° | conf={pose.tracker_confidence} | "
                f"{param_state_str()}",
                end=""
            )

    except KeyboardInterrupt:
        print("\n停止。")

    finally:
        running = False
        t.join(timeout=1.0)     # 让键盘线程恢复终端设置

        # ---- 退出保险: 哪个参数处于测试值就自动恢复哪个 ----
        for p_ in PARAMS:
            name = p_['name']
            v = P['cur'][name]
            if v is not None and abs(v - p_['off']) < 1e-4 \
                    and abs(p_['off'] - P['on_value'][name]) > 1e-4:
                print(f"\n退出前自动恢复 {name} 到 {P['on_value'][name]:.3f} ...")
                if set_param_blocking(master, name, P['on_value'][name]):
                    print(f"{name} 已恢复。")
                else:
                    print(f"!! {name} 自动恢复失败! 请重跑本脚本按 b, 或在 QGC 把 "
                          f"{name} 改回 {P['on_value'][name]:.3f} !!")
        if pipe is not None:
            pipe.stop()
        master.close()


if __name__ == '__main__':
    main()
