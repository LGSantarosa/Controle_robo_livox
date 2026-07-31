"""UMA rajada curta gravando as DUAS rodas + pose do LIO. Volta a zero e para."""
import csv, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

V, WZ, DUR, CSVP = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
ASSENTA, COAST = 1.0, 2.5

def yaw_de(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class R(Node):
    def __init__(self):
        super().__init__('rajada_rodas')
        self.pub = self.create_publisher(TwistStamped,'/hoverboard_base_controller/cmd_vel',10)
        self.yaw = self.x = self.y = None
        self.ve = self.vd = 0.0
        self.create_subscription(Odometry,'/Odometry', self.cb, 10)
        self.create_subscription(Float64,'/hoverboard/left_wheel/velocity',
                                 lambda m: setattr(self,'ve',m.data), 20)
        self.create_subscription(Float64,'/hoverboard/right_wheel/velocity',
                                 lambda m: setattr(self,'vd',m.data), 20)
    def cb(self, m):
        p = m.pose.pose
        self.x, self.y, self.yaw = p.position.x, p.position.y, yaw_de(p.orientation)
    def manda(self, v, wz):
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v); m.twist.angular.z = float(wz); self.pub.publish(m)

rclpy.init(); n = R()
t0 = time.time()
while time.time()-t0 < ASSENTA: rclpy.spin_once(n, timeout_sec=0.02)
reg = []
t0 = time.time()
try:
    while time.time()-t0 < DUR:
        n.manda(V, WZ); rclpy.spin_once(n, timeout_sec=0.02)
        reg.append((time.time()-t0, V, WZ, n.ve, n.vd, n.x, n.y, n.yaw))
finally:
    # Zera E CONTINUA GRAVANDO. A versao anterior fazia estes 0,2 s num laco
    # separado, sem spin e sem registrar — e e justamente o trecho em que o robo
    # ainda esta em velocidade quase maxima logo apos o corte. A integral das
    # rodas perdia ~6 cm ali, o LIO (medido de ponta a ponta) nao perdia, e a
    # diferenca virava um "fator de escala de 1,25" que nao existe. 31-07.
    tz = time.time()
    while time.time()-tz < 0.2:
        n.manda(0.0, 0.0); rclpy.spin_once(n, timeout_sec=0.005)
        reg.append((DUR+time.time()-tz, 0.0, 0.0, n.ve, n.vd, n.x, n.y, n.yaw))
t0 = time.time()
while time.time()-t0 < COAST:
    rclpy.spin_once(n, timeout_sec=0.02)
    reg.append((DUR+0.2+time.time()-t0, 0.0, 0.0, n.ve, n.vd, n.x, n.y, n.yaw))
with open(CSVP,'w',newline='') as f:
    w = csv.writer(f); w.writerow(['t','cmd_v','cmd_wz','v_esq','v_dir','x','y','yaw']); w.writerows(reg)

print('COMANDO v=%+.2f wz=%+.2f por %.1f s   (%d amostras -> %s)' % (V, WZ, DUR, len(reg), CSVP))
print()
print('  t     cmd    v_esq    v_dir    |  diferenca')
for i in range(0, len(reg), max(1,len(reg)//18)):
    t,c,ve,vd = reg[i][0], (reg[i][1] or reg[i][2]), reg[i][3], reg[i][4]
    print('  %4.2f  %+.2f  %+7.3f  %+7.3f  |  %+7.3f' % (t, c, ve, vd, ve-vd))
rclpy.shutdown()
