# diag6.py - 诊断夹爪(ID=6)不转的问题
import sys, time
sys.path.append('scservo_sdk')
from scservo_sdk.port_handler import PortHandler
from scservo_sdk.sms_sts import sms_sts
from scservo_sdk.scservo_def import COMM_SUCCESS

ph = PortHandler('/dev/ttyACM0')
ph.openPort()
ph.setBaudRate(1000000)
handler = sms_sts(ph)

def rd2(reg):
    v, res, _ = handler.read2ByteTxRx(6, reg)
    return v if res == COMM_SUCCESS else None

def rd1(reg):
    v, res, _ = handler.read1ByteTxRx(6, reg)
    return v if res == COMM_SUCCESS else None

handler.write1ByteTxRx(6, 40, 1)   # 使能力矩
time.sleep(0.2)

# 1) 角度限位 (寄存器 9-12)
minL, maxL = rd2(9), rd2(11)
print('角度下限 min_limit =', minL, '  角度上限 max_limit =', maxL)

# 2) 当前位置
pos, res, err = handler.ReadPos(6)
print('当前位置 =', pos, '  读取:', 'OK' if res == COMM_SUCCESS else 'FAIL', '  错误码:', err)

# 3) 发移动命令
target = pos - 300
handler.WritePosEx(6, target, 1000, 50)
time.sleep(0.3)

# 4) 读回目标寄存器确认写入
goal = rd2(42)
print('发送目标 =', target, '  舵机目标寄存器 =', goal)

# 5) 1秒后读状态
time.sleep(1)
pos2, res, err2 = handler.ReadPos(6)
load = rd2(60)
volt = rd1(62)
temp = rd1(63)
moving = rd1(66)
print('1秒后位置 =', pos2, '  错误码:', err2)
print('负载 load =', load, '  (原始值, 越大越吃力)')
if volt is not None:
    print('电压 =', volt / 10.0, 'V')
print('温度 =', temp, '°C   运动中 moving =', moving)

handler.WritePosEx(6, pos, 1000, 50)   # 回原位
ph.closePort()