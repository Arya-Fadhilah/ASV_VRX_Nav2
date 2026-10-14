#!/usr/bin/env python3
"""
Alokasi thrust: geometry_msgs/Twist  ->  dua std_msgs/Float64.

Nav2 mengeluarkan /cmd_vel. ASV dua thruster tidak menerima Twist, melainkan
gaya per thruster. Node ini jembatannya.

Model yang dipakai (proporsional, sengaja sesederhana mungkin):

    F  = k_surge * vx          gaya maju total   (N)
    M  = k_yaw   * wz          momen yaw total   (N.m)

    kiri  = F/2 - M/B
    kanan = F/2 + M/B          B = jarak antar lambung (m)

Pembagian dengan B berasal dari definisi momen: M = (kanan - kiri) * B/2.

BATASAN YANG HARUS DISADARI:
  - Ini open loop. Kecepatan yang diminta Nav2 tidak dijamin tercapai;
    yang dijamin hanya arah dan proporsinya.
  - k_surge dan k_yaw diturunkan dari koefisien drag di plugin
    SimpleHydrodynamics, yang nilainya MASIH PLACEHOLDER. Jadi pemetaan
    "perintah 1 m/s menghasilkan 1 m/s" belum benar secara kuantitatif.
  - ASV underactuated: perintah vy (menyamping) diabaikan, memang tidak bisa.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class ThrustAllocator(Node):
    def __init__(self):
        super().__init__('thrust_allocator')

        self.declare_parameter('k_surge', 8.0)      # N per (m/s)
        self.declare_parameter('k_yaw', 4.0)        # N.m per (rad/s)
        self.declare_parameter('beam', 0.35)        # jarak antar lambung, m
        self.declare_parameter('max_thrust', 40.0)  # ikut max_thrust_cmd di xacro
        self.declare_parameter('min_thrust', -30.0)
        self.declare_parameter('cmd_timeout', 0.5)  # detik tanpa cmd_vel -> berhenti
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('left_topic',  '/wamv/thrusters/left/thrust')
        self.declare_parameter('right_topic', '/wamv/thrusters/right/thrust')

        g = lambda n: self.get_parameter(n).value
        self.k_surge, self.k_yaw, self.beam = g('k_surge'), g('k_yaw'), g('beam')
        self.hi, self.lo = g('max_thrust'), g('min_thrust')
        self.timeout = g('cmd_timeout')

        self.vx = 0.0
        self.wz = 0.0
        self.last_cmd = None
        self.warned_vy = False

        self.pub_l = self.create_publisher(Float64, g('left_topic'), 10)
        self.pub_r = self.create_publisher(Float64, g('right_topic'), 10)
        self.create_subscription(Twist, g('cmd_vel_topic'), self.on_cmd, 10)
        self.create_timer(1.0 / g('rate'), self.tick)

        self.get_logger().info(
            f"alokasi thrust aktif | k_surge={self.k_surge} k_yaw={self.k_yaw} "
            f"beam={self.beam} m | batas [{self.lo}, {self.hi}] N")
        self.get_logger().info(
            f"  {g('cmd_vel_topic')} -> {g('left_topic')} , {g('right_topic')}")

    def on_cmd(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z
        self.last_cmd = self.get_clock().now()
        if abs(msg.linear.y) > 1e-6 and not self.warned_vy:
            self.get_logger().warn(
                'cmd_vel memuat linear.y - diabaikan. ASV dua thruster tetap '
                'tidak bisa bergerak menyamping. Pastikan controller Nav2 '
                'dikonfigurasi sebagai differential, bukan omni.')
            self.warned_vy = True

    def tick(self):
        stale = (self.last_cmd is None or
                 (self.get_clock().now() - self.last_cmd).nanoseconds * 1e-9 > self.timeout)
        vx, wz = (0.0, 0.0) if stale else (self.vx, self.wz)

        F = self.k_surge * vx
        M = self.k_yaw * wz
        left = F / 2.0 - M / self.beam
        right = F / 2.0 + M / self.beam

        clamp = lambda v: max(self.lo, min(self.hi, v))
        self.pub_l.publish(Float64(data=clamp(left)))
        self.pub_r.publish(Float64(data=clamp(right)))


def main():
    rclpy.init()
    n = ThrustAllocator()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            n.pub_l.publish(Float64(data=0.0))
            n.pub_r.publish(Float64(data=0.0))
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
