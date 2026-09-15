import csv, sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
rclpy.init(); n = rclpy.create_node("grava_reta")
f = open(sys.argv[1], "w", newline=""); w = csv.writer(f)
w.writerow(["t", "topico", "FL", "FR", "dpad_x"])
t0 = time.time()
n.create_subscription(Float64MultiArray, "hoverboard/wheel_velocities",
    lambda m: w.writerow([f"{time.time()-t0:.3f}", "rodas", m.data[0], m.data[1], ""]), qos_profile_sensor_data)
n.create_subscription(Twist, "dpad_vel",
    lambda m: w.writerow([f"{time.time()-t0:.3f}", "dpad", "", "", m.linear.x]), 10)
fim = time.time() + float(sys.argv[2])
while time.time() < fim: rclpy.spin_once(n, timeout_sec=0.1); f.flush()
f.close()
