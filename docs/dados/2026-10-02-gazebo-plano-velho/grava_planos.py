"""Gravador passivo: planos publicados x aceitos, com a pose do robô."""
import csv, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry

rclpy.init(); n = Node('grava_planos')
w = csv.writer(open(sys.argv[1], 'w'))
w.writerow(['t', 'topico', 'n', 'comp', 'x0', 'y0', 'xf', 'yf', 'rx', 'ry',
            'd_inicio', 'd_ao_plano'])
pose = [None]
def cb_odom(m):
    pose[0] = (m.pose.pose.position.x, m.pose.pose.position.y)
def cb(top):
    def f(m):
        p = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
        if len(p) < 2:
            return
        comp = sum(math.dist(a, b) for a, b in zip(p, p[1:]))
        rx, ry = pose[0] if pose[0] else (float('nan'),) * 2
        d0 = math.dist(p[0], (rx, ry))
        dp = min(math.dist(q, (rx, ry)) for q in p)
        w.writerow([f'{n.get_clock().now().nanoseconds*1e-9:.3f}', top, len(p),
                    f'{comp:.3f}', f'{p[0][0]:.3f}', f'{p[0][1]:.3f}',
                    f'{p[-1][0]:.3f}', f'{p[-1][1]:.3f}', f'{rx:.3f}',
                    f'{ry:.3f}', f'{d0:.3f}', f'{dp:.3f}'])
    return f
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(Odometry, '/Odometry', cb_odom, 10)
for t in ('/plan', '/plan_smoothed', '/path_follower/plano_aceito'):
    n.create_subscription(Path, t, cb(t), q)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
