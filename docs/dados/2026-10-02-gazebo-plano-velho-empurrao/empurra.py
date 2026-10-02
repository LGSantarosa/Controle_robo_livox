"""Desvio provocado pelo teclado (/key_vel): gira à esquerda, anda até sair
~0,30 m da linha que seguia, e solta com zero explícito."""
import math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

X_GATILHO, GIRO, DESVIO = 5.0, math.radians(70), 0.30
rclpy.init(); n = Node('empurra')
pub = n.create_publisher(TwistStamped, '/key_vel', 10)
st = {}
def cb(m):
    q = m.pose.pose.orientation
    st['x'], st['y'] = m.pose.pose.position.x, m.pose.pose.position.y
    st['yaw'] = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
n.create_subscription(Odometry, '/Odometry', cb, 10)
def manda(v, wz):
    m = TwistStamped(); m.header.frame_id = 'base_link'
    m.twist.linear.x, m.twist.angular.z = v, wz; pub.publish(m)
def log(s): print(f'{time.time():.3f} {s}', flush=True)
def spin(): rclpy.spin_once(n, timeout_sec=0.05)

t_lim = time.time() + 120
while time.time() < t_lim and not ('x' in st and st['x'] > X_GATILHO and st['y'] < 3.2):
    spin()
if 'x' not in st or st['x'] <= X_GATILHO:
    log('gatilho nunca veio; nada publicado'); sys.exit(1)
x0, y0, h0 = st['x'], st['y'], st['yaw']
log(f'GATILHO pose ({x0:.2f},{y0:.2f}) rumo {math.degrees(h0):.0f}')
fim = time.time() + 4.0
while time.time() < fim:
    d = math.atan2(math.sin(st['yaw']-h0), math.cos(st['yaw']-h0))
    if d >= GIRO: break
    manda(0.0, 1.0); spin()
log(f'girou {math.degrees(st["yaw"]-h0):.0f} graus')
fim = time.time() + 4.0
def lateral():   # afastamento perpendicular à linha que ele seguia
    return -(st['x']-x0)*math.sin(h0) + (st['y']-y0)*math.cos(h0)
while time.time() < fim and lateral() < DESVIO:
    manda(0.25, 0.0); spin()
log(f'SOLTOU com afastamento {lateral():.2f} m, pose ({st["x"]:.2f},{st["y"]:.2f})')
fim = time.time() + 0.5
while time.time() < fim:
    manda(0.0, 0.0); spin()
fim = time.time() + 2.0
while time.time() < fim: spin()
log(f'2 s depois: pose ({st["x"]:.2f},{st["y"]:.2f}) afastamento {lateral():.2f} m')
