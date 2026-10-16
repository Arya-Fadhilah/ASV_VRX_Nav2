#!/usr/bin/env python3
# -----------------------------------------------------------------------------
#  gps_waypoint_follower.py
#
#  Mengubah daftar waypoint lat/lon menjadi pose di frame `map`, lalu
#  mengirimkannya ke action FollowWaypoints milik Nav2.
#
#  KENAPA BEGINI, BUKAN /follow_gps_waypoints:
#  Nav2 Humble TIDAK punya FollowGPSWaypoints. Sudah kuperiksa langsung di
#  branch humble (nav2_msgs 1.1.20): daftar action-nya hanya FollowWaypoints,
#  NavigateToPose, NavigateThroughPoses, ComputePath*, FollowPath, Spin,
#  BackUp, DriveOnHeading, AssistedTeleop, Wait, SmoothPath, DummyBehavior.
#  Tutorial GPS di docs.nav2.org itu untuk rolling. Jadi konversinya harus
#  dilakukan di sisi kita, persis seperti yang dilakukan YILDIZ-USV.
#
#  KENAPA TIDAK PAKAI robot_localization / navsat_transform_node:
#  Arsitektur TF proyek ini (Opsi B) menetapkan ORB-SLAM3 yang mengisi
#  map->odom DAN odom->base_link, tanpa EKF. navsat_transform + EKF global
#  akan menerbitkan transform yang bertabrakan dengan itu, dan GPS akan
#  diam-diam ikut menentukan posisi kapal. Node ini sengaja TIDAK menerbitkan
#  TF apa pun. Dia cuma membaca GPS sekali untuk datum, lalu berhitung.
#
#  KENAPA BUKAN UTM:
#  YILDIZ memakai utm.from_latlon(). UTM punya konvergensi grid, yaitu selisih
#  sudut antara utara-grid dan utara-sebenarnya, yang tidak nol kecuali tepat
#  di meridian pusat zona. Gazebo memakai bidang singgung ENU lokal di titik
#  <spherical_coordinates>, bukan UTM. Jadi di sini dipakai konversi ENU lokal
#  (flat-earth WGS84), yang cocok dengan Gazebo dan tidak butuh paket `utm`.
# -----------------------------------------------------------------------------

import math
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import FollowWaypoints

# WGS84
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_enu(lat, lon, lat0, lon0):
    """Lat/lon (derajat) -> (east, north) meter, relatif terhadap (lat0, lon0).

    Pendekatan bidang singgung lokal. Galatnya di bawah beberapa sentimeter
    untuk jarak sampai sekitar 10 km, jauh lebih kecil daripada derau GPS.
    """
    lat0_rad = math.radians(lat0)
    sin_lat0 = math.sin(lat0_rad)
    denom = 1.0 - WGS84_E2 * sin_lat0 * sin_lat0

    # radius lengkung vertikal utama (arah timur-barat)
    r_n = WGS84_A / math.sqrt(denom)
    # radius lengkung meridian (arah utara-selatan)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)

    east = math.radians(lon - lon0) * r_n * math.cos(lat0_rad)
    north = math.radians(lat - lat0) * r_m
    return east, north


class GpsWaypointFollower(Node):

    def __init__(self):
        super().__init__('gps_waypoint_follower')

        self.declare_parameter('fix_topic', '/wamv/sensors/gps/gps/fix')
        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('goal_frame', 'map')
        # 0.0/0.0 berarti: pakai fix GPS pertama sebagai datum.
        self.declare_parameter('datum_lat', 0.0)
        self.declare_parameter('datum_lon', 0.0)
        # Tunggu beberapa fix sebelum mengunci datum, supaya tidak memakai
        # pesan pertama yang kadang belum settle.
        self.declare_parameter('datum_samples', 10)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('auto_send', True)

        self.fix_topic = self.get_parameter('fix_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.datum_samples = int(self.get_parameter('datum_samples').value)
        self.auto_send = bool(self.get_parameter('auto_send').value)

        self.datum_lat = float(self.get_parameter('datum_lat').value)
        self.datum_lon = float(self.get_parameter('datum_lon').value)
        self.datum_locked = (self.datum_lat != 0.0 or self.datum_lon != 0.0)

        self._fix_buf = []
        self._sent = False

        self.waypoints_ll = self._load_waypoints()
        if not self.waypoints_ll:
            self.get_logger().error(
                'Tidak ada waypoint yang terbaca. Set parameter waypoint_file.')

        # GPS di Gazebo terbit BEST_EFFORT lewat bridge. Kalau dipaksa
        # RELIABLE, langganannya tidak akan pernah cocok dan node ini
        # menunggu selamanya tanpa pesan error.
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST)
        self.sub_fix = self.create_subscription(
            NavSatFix, self.fix_topic, self.on_fix, qos)

        self.pub_markers = None
        if bool(self.get_parameter('publish_markers').value):
            self.pub_markers = self.create_publisher(
                MarkerArray, '~/waypoints', 1)

        self.ac = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        if self.datum_locked:
            self.get_logger().info(
                f'Datum dari parameter: lat={self.datum_lat:.8f} '
                f'lon={self.datum_lon:.8f}')
            self._on_datum_ready()
        else:
            self.get_logger().info(
                f'Menunggu {self.datum_samples} fix GPS di {self.fix_topic} '
                f'untuk mengunci datum...')

    # ---------------------------------------------------------------- waypoint
    def _load_waypoints(self):
        path = self.get_parameter('waypoint_file').value
        if not path:
            return []
        if not os.path.exists(path):
            self.get_logger().error(f'waypoint_file tidak ditemukan: {path}')
            return []
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        wps = data.get('waypoints', [])
        out = []
        for i, wp in enumerate(wps):
            try:
                out.append((float(wp['latitude']), float(wp['longitude'])))
            except (KeyError, TypeError, ValueError) as e:
                self.get_logger().error(f'Waypoint ke-{i} tidak valid: {e}')
        # datum boleh ikut ditulis di file yang sama
        d = data.get('datum')
        if d and not self.datum_locked:
            try:
                self.datum_lat = float(d['latitude'])
                self.datum_lon = float(d['longitude'])
                self.datum_locked = True
                self.get_logger().info('Datum diambil dari waypoint_file.')
            except (KeyError, TypeError, ValueError):
                pass
        return out

    # -------------------------------------------------------------------- GPS
    def on_fix(self, msg: NavSatFix):
        if self.datum_locked:
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self._fix_buf.append((msg.latitude, msg.longitude))
        if len(self._fix_buf) < self.datum_samples:
            return
        n = float(len(self._fix_buf))
        self.datum_lat = sum(p[0] for p in self._fix_buf) / n
        self.datum_lon = sum(p[1] for p in self._fix_buf) / n
        self.datum_locked = True
        self.get_logger().info(
            f'Datum terkunci dari rata-rata {int(n)} fix: '
            f'lat={self.datum_lat:.8f} lon={self.datum_lon:.8f}')
        self._on_datum_ready()

    # --------------------------------------------------------------- konversi
    def _on_datum_ready(self):
        self.poses = self._build_poses()
        self._log_table()
        if self.pub_markers is not None:
            self.pub_markers.publish(self._build_markers())
        if self.auto_send and self.poses and not self._sent:
            self._sent = True
            self.send_goal()

    def _build_poses(self):
        xy = [geodetic_to_enu(lat, lon, self.datum_lat, self.datum_lon)
              for lat, lon in self.waypoints_ll]
        poses = []
        now = self.get_clock().now().to_msg()
        for i, (x, y) in enumerate(xy):
            # Heading diarahkan ke waypoint berikutnya. Untuk kapal, orientasi
            # di titik tujuan jarang penting, tapi Nav2 tetap butuh pose penuh
            # dan RPP memakai heading ini saat rotate-to-goal.
            if i + 1 < len(xy):
                yaw = math.atan2(xy[i + 1][1] - y, xy[i + 1][0] - x)
            elif i > 0:
                yaw = math.atan2(y - xy[i - 1][1], x - xy[i - 1][0])
            else:
                yaw = 0.0
            p = PoseStamped()
            p.header.frame_id = self.goal_frame
            p.header.stamp = now
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.z = math.sin(yaw / 2.0)
            p.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(p)
        return poses

    def _log_table(self):
        self.get_logger().info(
            f'Konversi {len(self.poses)} waypoint ke frame "{self.goal_frame}":')
        self.get_logger().info('  idx |      lat      |      lon      |      x |      y')
        for i, (p, (lat, lon)) in enumerate(zip(self.poses, self.waypoints_ll)):
            self.get_logger().info(
                f'  {i:3d} | {lat:13.8f} | {lon:13.8f} | '
                f'{p.pose.position.x:6.2f} | {p.pose.position.y:6.2f}')

    def _build_markers(self):
        arr = MarkerArray()
        for i, p in enumerate(self.poses):
            m = Marker()
            m.header = p.header
            m.ns = 'gps_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = p.pose
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 0.9
            arr.markers.append(m)

            t = Marker()
            t.header = p.header
            t.ns = 'gps_waypoints_label'
            t.id = i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose = p.pose
            t.pose.position.z = 1.0
            t.scale.z = 0.5
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = str(i)
            arr.markers.append(t)
        return arr

    # ----------------------------------------------------------------- action
    def send_goal(self):
        self.get_logger().info('Menunggu action server follow_waypoints...')
        if not self.ac.wait_for_server(timeout_sec=20.0):
            self.get_logger().error(
                'follow_waypoints tidak tersedia. Nav2 sudah jalan?')
            return
        goal = FollowWaypoints.Goal()
        goal.poses = self.poses
        self.get_logger().info(f'Mengirim {len(self.poses)} waypoint.')
        fut = self.ac.send_goal_async(goal, feedback_callback=self.on_feedback)
        fut.add_done_callback(self.on_goal_response)

    def on_feedback(self, fb):
        self.get_logger().info(
            f'Menuju waypoint #{fb.feedback.current_waypoint}')

    def on_goal_response(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Goal ditolak Nav2.')
            return
        self.get_logger().info('Goal diterima.')
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, fut):
        res = fut.result().result
        missed = list(res.missed_waypoints)
        if missed:
            self.get_logger().warn(f'Waypoint terlewat: {missed}')
        else:
            self.get_logger().info('Semua waypoint tercapai.')


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()