#!/usr/bin/env python3
# -----------------------------------------------------------------------------
#  gps_waypoint_interactive.py
#
#  Versi interaktif: klik titik di RViz, kapal menuju ke sana, dan titiknya
#  otomatis dicatat sebagai lat/lon ke file YAML supaya misi yang sama bisa
#  diulang persis di kemudian hari.
#
#  BEDANYA DENGAN TUTORIAL NAV2:
#  Tutorial Nav2 (rolling) memakai mapviz: kamu klik di peta satelit pada
#  frame wgs84, titiknya dikirim ke action /follow_gps_waypoints. Dua-duanya
#  tidak tersedia di sini. /follow_gps_waypoints tidak ada di Humble (sudah
#  diverifikasi langsung di nav2_msgs 1.1.20), dan mapviz menambah satu lagi
#  jendela OpenGL di atas llvmpipe yang RTF-nya sudah 3-6%.
#
#  Jadi arah konversinya dibalik. Kamu klik di RViz (frame map, yang sudah
#  terbuka dan tidak menambah beban), node ini mengubah x,y -> lat/lon dengan
#  datum yang sama, mengirim kapal ke sana lewat NavigateToPose, dan menulis
#  lat/lon-nya ke file. Hasil akhirnya identik: daftar waypoint dalam lat/lon
#  yang bisa dipakai ulang di simulasi maupun di lapangan.
#
#  CARA PAKAI DI RVIZ:
#  Pakai tool "Publish Point" di toolbar (bukan "Nav2 Goal"). Tool itu
#  menerbitkan geometry_msgs/PointStamped ke /clicked_point pada frame yang
#  sedang jadi Fixed Frame. Pastikan Fixed Frame = map.
# -----------------------------------------------------------------------------

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def _radii(lat0_deg):
    lat0 = math.radians(lat0_deg)
    s = math.sin(lat0)
    denom = 1.0 - WGS84_E2 * s * s
    r_n = WGS84_A / math.sqrt(denom)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 1.5)
    return r_n, r_m, lat0


def enu_to_geodetic(east, north, lat0, lon0):
    """(east, north) meter -> (lat, lon) derajat. Kebalikan dari konversi
    di gps_waypoint_follower.py, memakai bidang singgung lokal yang sama."""
    r_n, r_m, lat0_rad = _radii(lat0)
    lat = lat0 + math.degrees(north / r_m)
    lon = lon0 + math.degrees(east / (r_n * math.cos(lat0_rad)))
    return lat, lon


class GpsWaypointInteractive(Node):

    def __init__(self):
        super().__init__('gps_waypoint_interactive')

        self.declare_parameter('datum_lat', 0.0)
        self.declare_parameter('datum_lon', 0.0)
        self.declare_parameter('waypoint_file', '')      # untuk membaca datum
        self.declare_parameter('output_file', '')        # tempat mencatat klik
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('click_topic', '/clicked_point')
        # false = hanya mencatat, kapal tidak bergerak. Berguna saat kamu
        # ingin menyusun daftar waypoint dulu tanpa menjalankan misi.
        self.declare_parameter('send_on_click', True)

        self.goal_frame = self.get_parameter('goal_frame').value
        self.send_on_click = bool(self.get_parameter('send_on_click').value)
        self.output_file = self.get_parameter('output_file').value or \
            os.path.expanduser('~/vrx_ws/gps_waypoints_logged.yaml')

        self.datum_lat = float(self.get_parameter('datum_lat').value)
        self.datum_lon = float(self.get_parameter('datum_lon').value)
        self._load_datum_from_file()

        if self.datum_lat == 0.0 and self.datum_lon == 0.0:
            self.get_logger().error(
                'Datum belum diset. Isi datum_lat/datum_lon, atau tunjuk '
                'waypoint_file ke gps_waypoints.yaml yang sudah punya datum. '
                'Tanpa datum, konversi ke lat/lon tidak mungkin.')
            raise SystemExit(1)

        self.get_logger().info(
            f'Datum: lat={self.datum_lat:.8f} lon={self.datum_lon:.8f}')

        self.logged = []
        self._load_existing_log()

        self.sub = self.create_subscription(
            PointStamped, self.get_parameter('click_topic').value,
            self.on_click, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '~/clicked', 1)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            'Siap. Di RViz: set Fixed Frame = map, pilih tool "Publish Point", '
            'lalu klik di peta.')
        if not self.send_on_click:
            self.get_logger().info(
                'send_on_click=false: klik hanya dicatat, kapal tidak bergerak.')
        self.get_logger().info(f'Catatan ditulis ke: {self.output_file}')

    # ------------------------------------------------------------------ datum
    def _load_datum_from_file(self):
        if self.datum_lat != 0.0 or self.datum_lon != 0.0:
            return
        path = self.get_parameter('waypoint_file').value
        if not path or not os.path.exists(path):
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
            d = data.get('datum')
            if d:
                self.datum_lat = float(d['latitude'])
                self.datum_lon = float(d['longitude'])
        except Exception as e:
            self.get_logger().warn(f'Gagal membaca datum dari {path}: {e}')

    def _load_existing_log(self):
        if not os.path.exists(self.output_file):
            return
        try:
            with open(self.output_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            for wp in data.get('waypoints', []):
                self.logged.append((float(wp['latitude']),
                                    float(wp['longitude'])))
            if self.logged:
                self.get_logger().info(
                    f'Melanjutkan catatan lama: {len(self.logged)} waypoint.')
        except Exception as e:
            self.get_logger().warn(f'Gagal membaca {self.output_file}: {e}')

    # ------------------------------------------------------------------ klik
    def on_click(self, msg: PointStamped):
        frame = msg.header.frame_id
        if frame and frame != self.goal_frame:
            self.get_logger().warn(
                f'Klik datang di frame "{frame}", bukan "{self.goal_frame}". '
                f'Ubah Fixed Frame RViz jadi {self.goal_frame}, '
                f'kalau tidak koordinatnya salah.')
            return

        x, y = msg.point.x, msg.point.y
        lat, lon = enu_to_geodetic(x, y, self.datum_lat, self.datum_lon)
        self.logged.append((lat, lon))

        self.get_logger().info(
            f'#{len(self.logged)-1}  x={x:7.2f} y={y:7.2f}  ->  '
            f'lat={lat:.8f} lon={lon:.8f}')

        self._write_log()
        self._publish_markers()

        if self.send_on_click:
            self._send(x, y)

    def _write_log(self):
        data = {
            'datum': {'latitude': float(self.datum_lat),
                      'longitude': float(self.datum_lon)},
            'waypoints': [{'latitude': float(a), 'longitude': float(b)}
                          for a, b in self.logged],
        }
        tmp = self.output_file + '.tmp'
        try:
            with open(tmp, 'w') as f:
                yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
                f.flush()
                os.fsync(f.fileno())
            os.replace(tmp, self.output_file)
        except Exception as e:
            self.get_logger().error(f'Gagal menulis {self.output_file}: {e}')

    # ----------------------------------------------------------------- action
    def _send(self, x, y):
        if not self.ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'navigate_to_pose tidak tersedia. Nav2 sudah jalan?')
            return
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = self.goal_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.w = 1.0
        goal.pose = p
        self.get_logger().info(f'Mengirim goal ke x={x:.2f} y={y:.2f}')
        self.ac.send_goal_async(goal).add_done_callback(self._on_response)

    def _on_response(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().error('Goal ditolak Nav2.')
            return
        h.get_result_async().add_done_callback(
            lambda f: self.get_logger().info('Goal selesai.'))

    # ---------------------------------------------------------------- marker
    def _publish_markers(self):
        arr = MarkerArray()
        r_n, r_m, lat0 = _radii(self.datum_lat)
        for i, (lat, lon) in enumerate(self.logged):
            x = math.radians(lon - self.datum_lon) * r_n * math.cos(lat0)
            y = math.radians(lat - self.datum_lat) * r_m
            m = Marker()
            m.header.frame_id = self.goal_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'clicked_gps'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.1, 0.9
            arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GpsWaypointInteractive()
    except SystemExit:
        rclpy.shutdown()
        return
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