"""Gira ate o LIO acusar 180° e para. So o LIO decide o corte."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

WZ, ALVO, TETO = 0.25, math.pi, 20.0
def yaw_de(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class T(Node):
    def __init__(self):
        super().__init__('meia_lio')
        self.pub=self.create_publisher(TwistStamped,'/hoverboard_base_controller/cmd_vel',10)
        self.yaw=None; self.acc=0.0; self.ve=self.vd=0.0; self.wroda=0.0; self.tw=None
        self.create_subscription(Odometry,'/Odometry', self.cb, 10)
        self.create_subscription(Float64,'/hoverboard/left_wheel/velocity',
                                 lambda m: setattr(self,'ve',m.data), 20)
        self.create_subscription(Float64,'/hoverboard/right_wheel/velocity', self.cbd, 20)
    def cb(self,m):
        y=yaw_de(m.pose.pose.orientation)
        if self.yaw is not None:
            d=y-self.yaw
            while d>math.pi: d-=2*math.pi
            while d<-math.pi: d+=2*math.pi
            self.acc+=d
        self.yaw=y
    def cbd(self,m):
        self.vd=m.data
        t=time.time()
        if self.tw is not None:
            self.wroda += ((self.vd-self.ve)*0.080/0.270)*(t-self.tw)
        self.tw=t
    def manda(self,wz):
        m=TwistStamped(); m.header.stamp=self.get_clock().now().to_msg()
        m.twist.angular.z=float(wz); self.pub.publish(m)

rclpy.init(); n=T()
t0=time.time()
while time.time()-t0<1.5: rclpy.spin_once(n,timeout_sec=0.02)
n.acc=0.0; n.wroda=0.0
motivo='LIO acusou 180°'
t0=time.time()
try:
    while abs(n.acc)<ALVO:
        if time.time()-t0>TETO: motivo='TETO DE TEMPO'; break
        n.manda(WZ); rclpy.spin_once(n,timeout_sec=0.02)
finally:
    for _ in range(12): n.manda(0.0); time.sleep(0.02)
tc=time.time()
while time.time()-tc<3.0: rclpy.spin_once(n,timeout_sec=0.02)
print('cortado por: %s'%motivo)
print()
print('  no CORTE     -> LIO %+.1f°   rodas %+.1f°'%(math.degrees(n.acc), math.degrees(n.wroda)))
print('  (o robo ainda gira na inercia depois do corte)')
print('  ao ASSENTAR  -> LIO %+.1f°   rodas %+.1f°'%(math.degrees(n.acc), math.degrees(n.wroda)))
rclpy.shutdown()
