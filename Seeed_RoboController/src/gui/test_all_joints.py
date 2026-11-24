# test_all_joints.py - 逐个验证 1~5 号关节能转动
# 每个关节：读限位 → 读当前位置 → 小幅移动(±120计数≈±10.5°) → 回原位
import sys, time
sys.path.append('scservo_sdk')
from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts
from scservo_sdk.scservo_def import COMM_SUCCESS

ph = PortHandler('/dev/ttyACM0')
ph.openPort()
ph.setBaudRate(1000000)
handler = sms_sts(ph)

# 1~5 号关节 (夹爪6已验证)
for sid in range(1, 6):
    handler.write1ByteTxRx(sid, 40, 1)          # 使能力矩
    time.sleep(0.1)

    # 读限位和当前位置
    minL, res1, _ = handler.read2ByteTxRx(sid, 9)
    maxL, res2, _ = handler.read2ByteTxRx(sid, 11)
    pos, res, _ = handler.ReadPos(sid)
    print(f'\n=== ID={sid} ===')
    print(f'  限位: {minL} ~ {maxL}   当前位置: {pos}')

    if res != COMM_SUCCESS or minL is None:
        print('  读取失败，跳过')
        continue

    # 目标 = 当前 + 120（往增大方向，通常安全），若超出限位则反向
    target = pos + 120
    if target > maxL:
        target = pos - 120
    if target < minL:
        print('  限位太窄无法移动 ±120，跳过')
        continue

    handler.WritePosEx(sid, target, 1000, 50)
    time.sleep(2)
    pos2, _, _ = handler.ReadPos(sid)
    print(f'  移到 {target} -> 实测 {pos2}')

    handler.WritePosEx(sid, pos, 1000, 50)      # 回原位
    time.sleep(2)
    pos3, _, _ = handler.ReadPos(sid)
    print(f'  回位 -> 实测 {pos3}')

ph.closePort()
print('\n全部测试完成')