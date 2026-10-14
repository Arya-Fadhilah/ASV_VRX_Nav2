# ASV + VRX + Nav2 — Cara Menjalankan

Simulasi ASV kustom di VRX (Gazebo Garden), dengan Nav2 menggerakkan kapal
lewat `/cmd_vel` → alokasi thrust → thruster kiri/kanan.

---

## Build

```bash
cd ~/vrx_ws
colcon build --symlink-install
source install/setup.bash
```

Build satu paket saja:

```bash
colcon build --packages-select asv_control --symlink-install
colcon build --packages-select asv_description --symlink-install
```

---

## Menjalankan

Setiap terminal baru wajib di-source dulu:

```bash
source ~/vrx_ws/install/setup.bash
```

Urutannya penting: Terminal 1 → 3 → 4. Nav2 butuh TF dan `/odom` sudah ada
sebelum ia aktif. RViz2 boleh dijalankan kapan saja.

### Terminal 1 — Simulasi

```bash
ros2 launch vrx_gz competition.launch.py \
  world:=$HOME/vrx_ws/src/asv_description/worlds/lake \
  urdf:=$(ros2 pkg prefix --share asv_description)/urdf/asv.urdf.xacro
```

Tanpa GUI Gazebo (lebih ringan, pantau lewat RViz2 saja):

```bash
ros2 launch vrx_gz competition.launch.py \
  world:=$HOME/vrx_ws/src/asv_description/worlds/lake \
  urdf:=$(ros2 pkg prefix --share asv_description)/urdf/asv.urdf.xacro \
  headless:=true
```

### Terminal 2 — RViz2

```bash
ros2 launch nav2_bringup rviz_launch.py
```

### Terminal 3 — Alokasi cmd_vel ke thruster + odometry

```bash
ros2 launch asv_control asv_control.launch.py
```

Menjalankan dua node:
- `thrust_allocator` — `/cmd_vel` → `thrusters/{left,right}/thrust`
- `odom_from_sim` — `/wamv/pose` → `/odom`, `/odom_ground_truth`, TF `map → odom → wamv/base_link`

### Terminal 4 — Nav2

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$(ros2 pkg prefix --share asv_control)/config/nav2_asv.yaml
```

---

## Menggerakkan kapal

### Lewat RViz2 (satu goal)

Tool **Nav2 Goal** di toolbar, lalu klik di peta.

### Lewat RViz2 (banyak waypoint)

Buka panel: **Panels → Add New Panel → Navigation 2**

1. Klik **"Waypoint / Nav Through Poses Mode"**
2. Pakai tool **Nav2 Goal** berkali-kali untuk menumpuk titik
3. Klik **"Start Waypoint Following"** (berhenti di tiap titik)
   atau **"Start Nav Through Poses"** (jalur menerus, tidak berhenti)

Batalkan: tombol **"Cancel"** di panel yang sama.

### Teleop manual

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Matikan Nav2 dulu, kalau tidak `/cmd_vel` akan rebutan.

---

## Perintah berguna

Cek posisi dan kecepatan kapal:

```bash
ros2 topic echo /odom --field pose.pose.position
ros2 topic echo /odom --field twist.twist.linear.x
```

Cek perintah yang dikirim Nav2:

```bash
ros2 topic echo /cmd_vel
```

Cek thrust yang keluar:

```bash
ros2 topic echo /asv/thrusters/left/thrust
ros2 topic echo /asv/thrusters/right/thrust
```

Cek pohon TF:

```bash
ros2 run tf2_tools view_frames
```

Perintah kecepatan manual (tes kalibrasi, Nav2 harus mati):

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.6, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Batalkan goal dari terminal:

```bash
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, stamp: {sec: 0, nanosec: 0}}}"
```

Rekam data untuk evaluasi:

```bash
ros2 bag record -o run_01 /odom /odom_ground_truth /tf /tf_static /plan /cmd_vel
```

---

## File penting

| File | Isi |
|---|---|
| `src/asv_description/urdf/asv.urdf.xacro` | Model ASV: link, joint, massa, buoyancy, thruster, kamera, IMU |
| `src/asv_description/worlds/lake.sdf` | Dunia danau (gelombang dimatikan, `gain = 0`) |
| `src/asv_control/config/nav2_asv.yaml` | Parameter Nav2 |
| `src/asv_control/config/asv_control.yaml` | Parameter alokasi thrust dan odometry |
| `src/asv_control/asv_control/thrust_allocator.py` | `/cmd_vel` → thrust kiri/kanan |
| `src/asv_control/asv_control/odom_from_sim.py` | Pose Gazebo → `/odom` + TF |

---

## Catatan

- `k_surge` dan `k_yaw` di `asv_control.yaml` masih nilai sementara, belum
  dikalibrasi. Kapal bergerak lebih pelan dari yang diperintahkan Nav2.
- `xy_goal_tolerance: 0.8` di `nav2_asv.yaml` terlalu longgar untuk mengukur
  akurasi waypoint. Perketat setelah kalibrasi selesai.
- Global costmap masih `rolling_window: true` tanpa `static_layer`, karena
  ORB-SLAM3 belum menerbitkan `/map`.