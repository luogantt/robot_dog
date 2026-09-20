#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""S10 原地转角诊断：默认尝试左转 +90°。

和 axis_test.py、asdu_probe.py 放在同一目录。
默认 dry-run；显式加 --go 才真发。

核心：用 MotionStatus.AngularZ 对时间积分，不用固定时长猜 90°。
若 3 秒后净转角仍 <10°，自动停止，避免机器人只前后摆却一直发 Yaw。
"""
import argparse, math, os, socket, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from asdu_probe import MOTION_NAMES, build_heartbeat, parse
from axis_test import build_axis

MOTION_RL = 17

class Link:
    def __init__(self, host, port):
        self.target=(host,port)
        self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.sock.settimeout(0.03)
        self.motion_state=None; self.faults=[]
        self.linear_x=0.0; self.linear_y=0.0; self.angular_z=0.0
        self.motion_seq=0; self.motion_rx_time=None
    def heartbeat(self): self.sock.sendto(build_heartbeat(), self.target)
    def yaw(self,v): self.sock.sendto(build_axis('Yaw',float(v)), self.target)
    def pump(self):
        end=time.monotonic()+0.02
        while time.monotonic()<end:
            try: data,_=self.sock.recvfrom(65535)
            except (socket.timeout,OSError): break
            p=parse(data)
            if not p: continue
            _,_,items=p
            bs=items.get('BasicStatus')
            if isinstance(bs,dict): self.motion_state=bs.get('MotionState')
            ms=items.get('MotionStatus')
            if isinstance(ms,dict):
                for key,attr in [('LinearX','linear_x'),('LinearY','linear_y'),('AngularZ','angular_z')]:
                    v=ms.get(key)
                    if isinstance(v,(int,float)): setattr(self,attr,float(v))
                self.motion_seq+=1; self.motion_rx_time=time.monotonic()
            el=items.get('ErrorList')
            if isinstance(el,list): self.faults=el
    def worst(self):
        return max((max(f.get('Severities') or [0]) for f in self.faults),default=0)
    def stop(self,sec=1.0,hz=20.0):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            self.heartbeat(); self.yaw(0.0); self.pump(); time.sleep(1.0/hz)
    def close(self):
        try:self.sock.close()
        except OSError:pass

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--host',default='10.21.33.103')
    ap.add_argument('--port',type=int,default=30004)
    ap.add_argument('--degrees',type=float,default=90.0,help='正=左转，负=右转')
    ap.add_argument('--yaw',type=float,default=0.20)
    ap.add_argument('--slow-yaw',type=float,default=0.12)
    ap.add_argument('--slow-at',type=float,default=70.0)
    ap.add_argument('--timeout',type=float,default=8.0)
    ap.add_argument('--no-progress-time',type=float,default=3.0)
    ap.add_argument('--no-progress-deg',type=float,default=10.0)
    ap.add_argument('--max-linear',type=float,default=0.25)
    ap.add_argument('--hz',type=float,default=20.0)
    ap.add_argument('--go',action='store_true')
    a=ap.parse_args()
    if not (0<a.yaw<=0.30): raise SystemExit('--yaw 必须在 (0,0.30]')
    if not (0<a.slow_yaw<=a.yaw): raise SystemExit('--slow-yaw 必须 >0 且 <= --yaw')
    if not (1<=abs(a.degrees)<=180): raise SystemExit('--degrees 建议范围 1~180')
    direction=1.0 if a.degrees>0 else -1.0
    target=math.radians(abs(a.degrees))
    print(f"目标：{'左/逆时针' if direction>0 else '右/顺时针'} {abs(a.degrees):.1f}°")
    print(f"Yaw={direction*a.yaw:+.3f}，到 {a.slow_at:.0f}° 后降为 {direction*a.slow_yaw:+.3f}")
    print(f"超时 {a.timeout:.1f}s；{a.no_progress_time:.1f}s 后若净转角 < {a.no_progress_deg:.1f}° 则停止")
    if not a.go:
        print('[干跑] 没有发任何指令。确认周围安全后加 --go。'); return 0
    link=Link(a.host,a.port)
    try:
        for _ in range(40):
            link.heartbeat(); link.pump()
            if link.motion_state is not None: break
            time.sleep(0.05)
        print(f"MotionState={link.motion_state} ({MOTION_NAMES.get(link.motion_state,'未知')}) 最高故障等级={link.worst()}")
        if link.motion_state!=MOTION_RL: print('[拒绝] 不在 RL 控制(17)'); return 2
        if link.worst()>=5: print('[拒绝] 有 FATAL 故障'); return 2

        t0=time.monotonic(); last_beat=0.0; last_print=0.0; next_t=t0
        last_seq=link.motion_seq; last_sample_t=None; last_wz=None; angle=0.0
        reason='超时'
        while True:
            now=time.monotonic(); elapsed=now-t0; deg=math.degrees(angle)
            if elapsed>=a.timeout: reason='超时'; break
            if direction*angle>=target: reason=f'达到目标 {deg:+.1f}°'; break
            if elapsed>=a.no_progress_time and abs(deg)<a.no_progress_deg:
                reason=f'{elapsed:.1f}s 净转角仅 {deg:+.1f}°，纯 Yaw 未形成持续转身'; break
            if abs(link.linear_x)>a.max_linear or abs(link.linear_y)>a.max_linear:
                reason=f'意外平移过大 X={link.linear_x:+.3f} Y={link.linear_y:+.3f}'; break
            if now-last_beat>=0.9: link.heartbeat(); last_beat=now
            cmd=a.slow_yaw if abs(deg)>=a.slow_at else a.yaw
            link.yaw(direction*cmd); link.pump()
            if link.motion_seq!=last_seq and link.motion_rx_time is not None:
                st=link.motion_rx_time; wz=link.angular_z
                if last_sample_t is not None and last_wz is not None:
                    dt=st-last_sample_t
                    if 0<dt<0.5: angle += 0.5*(last_wz+wz)*dt
                last_sample_t=st; last_wz=wz; last_seq=link.motion_seq
            if now-last_print>=0.25:
                print(f"t={elapsed:4.1f}s  角度≈{math.degrees(angle):+6.1f}°  AngularZ={link.angular_z:+.3f}  X={link.linear_x:+.3f} Y={link.linear_y:+.3f}")
                last_print=now
            next_t += 1.0/a.hz
            sl=next_t-time.monotonic()
            if sl>0: time.sleep(sl)
            else: next_t=time.monotonic()
        print(f"停止原因：{reason}")
        print(f"积分得到净转角≈{math.degrees(angle):+.1f}°")
    finally:
        print('连续归零 ...')
        try: link.stop(1.0,20.0)
        finally: link.close()
    return 0

if __name__=='__main__':
    sys.exit(main())
