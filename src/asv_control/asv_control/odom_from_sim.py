#!/usr/bin/env python3
"""
Odometry dari ground truth simulasi.

VRX sama sekali tidak menerbitkan nav_msgs/Odometry, padahal Nav2 memerlukannya.
Node ini mengambil pose kapal yang sudah dijembatani VRX dan membungkusnya jadi
Odometry, plus menyiarkan TF odom -> base_link.

Ini memang "curang": posenya sempurna, tanpa derau dan tanpa drift. Untuk
menguji planner dan controller secara terisolasi, justru itu yang benar -
kamu sedang menguji perencanaan jalur, bukan estimator.

SUMBER YANG BISA DITUKAR
  publish_tf = true   -> node ini yang mengisi odom -> base_link
  publish_tf = false  -> ORB-SLAM3 yang mengisi; node ini tetap menerbitkan
                         /odom_ground_truth sebagai acuan pembanding.

Jadi saat ORB-SLAM3 sudah masuk, matikan publish_tf dan kamu langsung punya
dua lintasan untuk dibandingkan: estimasi SLAM versus kebenaran simulasi.
Selisihnya adalah data evaluasi, bukan sekadar debug.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomFromSim(Node):
    def __init__(self):
        super().__init__('odom_from_sim')

        self.declare_parameter('pose_topic', '/wamv/pose')
        self.declare_parameter('pose_child_frame', 'wamv')
        self.declare_parameter('base_frame', 'wamv/base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_tf', True)        # odom -> base_link
        self.declare_parameter('publish_map_odom', True)  # map -> odom identitas
        self.declare_parameter('vel_filter', 0.3)         # 0..1, makin kecil makin halus
        self.declare_parameter('zero_origin', True)

        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.src = g('pose_child_frame')
        self.odom_frame, self.map_frame = g('odom_frame'), g('map_frame')
        self.do_tf, self.alpha = g('publish_tf'), g('vel_filter')
        self.zero = g('zero_origin')
        self.origin = None

        self.prev = None          # (t, x, y, yaw)
        self.vx = self.vy = self.wz = 0.0
        self.seen_frames = False

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.pub_gt = self.create_publisher(Odometry, '/odom_ground_truth', 10)
        self.br = TransformBroadcaster(self)
        self.create_subscription(TFMessage, g('pose_topic'), self.on_pose, 20)

        if g('publish_map_odom'):
            # map -> odom identitas. Nav2 butuh rantai REP-105 utuh; tanpa SLAM
            # yang mengisinya, identitas sudah cukup untuk menguji planner.
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.static_br = StaticTransformBroadcaster(self, qos=qos)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.odom_frame
            t.transform.rotation.w = 1.0
            self.static_br.sendTransform(t)
            self.get_logger().info(f'{self.map_frame} -> {self.odom_frame} identitas disiarkan')

        self.get_logger().info(
            f"odom dari ground truth | pose={g('pose_topic')} base={self.base} "
            f"| publish_tf={self.do_tf}")

    @staticmethod
    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_pose(self, msg: TFMessage):
        tr = None
        for t in msg.transforms:
            if t.child_frame_id == self.src:
                tr = t
                break
        if tr is None:                      # cadangan
            for t in msg.transforms:
                if t.child_frame_id.endswith('base_link'):
                    tr = t
                    break
        if tr is None:
            if not self.seen_frames:
                names = [t.child_frame_id for t in msg.transforms]
                self.get_logger().warn(
                    f"frame '{self.src}' tidak ada di pesan pose. "
                    f"Yang tersedia: {names}. "
                    f"Setel parameter pose_child_frame ke salah satunya.")
                self.seen_frames = True
            return

        t_now = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        p, q = tr.transform.translation, tr.transform.rotation
        yaw = self.yaw_of(q)

        if self.zero and self.origin is None:
            self.origin = (p.x, p.y)
            self.get_logger().info(
                f'origin odom dinolkan di ({p.x:.1f}, {p.y:.1f})')
        ox, oy = self.origin if self.origin else (0.0, 0.0)

        # kecepatan dari beda maju, difilter ringan
        if self.prev is not None:
            dt = t_now - self.prev[0]
            if dt > 1e-4:
                dx, dy = p.x - self.prev[1], p.y - self.prev[2]
                dyaw = math.atan2(math.sin(yaw - self.prev[3]),
                                  math.cos(yaw - self.prev[3]))
                # kecepatan dunia -> frame badan
                c, s = math.cos(yaw), math.sin(yaw)
                vx_b = ( c * dx + s * dy) / dt
                vy_b = (-s * dx + c * dy) / dt
                a = self.alpha
                self.vx = a * vx_b + (1 - a) * self.vx
                self.vy = a * vy_b + (1 - a) * self.vy
                self.wz = a * (dyaw / dt) + (1 - a) * self.wz
        self.prev = (t_now, p.x, p.y, yaw)

        o = Odometry()
        o.header.stamp = tr.header.stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base
        o.pose.pose.position.x = p.x - ox
        o.pose.pose.position.y = p.y - oy
        o.pose.pose.position.z = p.z
        o.pose.pose.orientation = q
        o.twist.twist.linear.x = self.vx
        o.twist.twist.linear.y = self.vy
        o.twist.twist.angular.z = self.wz
        # kovarians kecil tapi bukan nol - ground truth, tapi Nav2 tidak suka nol murni
        for i in (0, 7, 14, 21, 28, 35):
            o.pose.covariance[i] = 1e-4
            o.twist.covariance[i] = 1e-4

        self.pub_gt.publish(o)
        if self.do_tf:
            self.pub.publish(o)
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = self.base
            t.transform.translation.x = p.x - ox
            t.transform.translation.y = p.y - oy
            t.transform.translation.z = p.z
            t.transform.rotation = q
            self.br.sendTransform(t)


def main():
    rclpy.init()
    n = OdomFromSim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
