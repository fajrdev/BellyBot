"""Shared helpers for headless Gazebo shot-camera capture of the storyboard videos.

Pipeline: `gz sim -s --headless-rendering` renders SDF camera sensors -> gz image
topics -> ros_gz_bridge (sensor_msgs/Image) -> OpenCV VideoWriter (mp4).

GZ_IP must be 127.0.0.1 on this host: the LAN interface drops multicast, which
stalls gz-transport discovery.
"""

import json
import math
import os
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
WS_INSTALL = REPO / "ros2_ws" / "install" / "setup.bash"
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")
SCRATCH = Path(os.environ.get("COMMANDCODE_SCRATCHPAD", "/tmp"))

SHOT_CAMERAS = {
    1: {
        "world": "entry_resolved",
        "mode": "entry",
        "cameras": {
            "camera_ingress": (2.8, -4.5, 1.4, 0.0, 0.28, 2.25),
            "camera_turn": (1.30, 0.00, 2.20, 0.0, 1.5708, 0.0),
            "camera_ramp": (5.20, 0.85, 0.95, 0.0, 0.12, -2.85),
        },
    },
    2: {
        "world": "transfer_cutaway",
        "mode": "transfer",
        "cameras": {
            "camera_corner_lift": (0.40, -1.80, 1.25, 0.0, 0.42, 1.12),
            "camera_buffer_travel": (2.10, -1.10, 1.10, 0.0, 0.35, 1.5708),
            "camera_tray_handoff": (4.40, -0.85, 0.95, 0.0, 0.30, 2.45),
        },
    },
    3: {
        "world": "transfer_cutaway",
        "mode": "transfer",
        "cameras": {
            "camera_reach_profile": (4.20, -1.65, 0.65, 0.0, 0.08, 1.5708),
            "camera_tray_pitch_lip": (4.95, -0.65, 0.22, 0.0, 0.15, 2.55),
            "camera_floor_placement": (5.35, -0.80, 0.55, 0.0, 0.22, 2.70),
        },
    },
    4: {
        "world": "transfer_cutaway",
        "mode": "transfer",
        "cameras": {
            "camera_retract_profile": (4.30, -1.50, 0.70, 0.0, 0.12, 1.5708),
            "camera_ceiling_clearance": (3.90, -1.80, 0.85, 0.0, 0.05, 1.5708),
            "camera_yaw_stack": (4.20, 0.00, 1.95, 0.0, 1.5708, 0.0),
        },
    },
    5: {
        "world": "transfer_cutaway",
        "mode": "transfer",
        "cameras": {
            "camera_spec_iso": (2.60, -3.80, 1.95, 0.0, 0.38, 1.95),
            "camera_pipeline_wide": (3.85, -2.40, 1.45, 0.0, 0.25, 1.5708),
            "camera_egress_front": (5.80, -0.40, 0.45, 0.0, 0.08, 3.05),
        },
    },
}


def ros_env():
    env = os.environ.copy()
    env["GZ_IP"] = "127.0.0.1"
    return env


def share_dir(package):
    out = subprocess.check_output(
        ["ros2", "pkg", "prefix", package], env=os.environ, text=True
    ).strip()
    return Path(out) / "share" / package


def resource_env(aircraft_share, desc_share):
    env = ros_env()
    paths = [
        str(aircraft_share / "models"),
        str(desc_share.parent),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([p for p in paths if p])
    return env


def camera_sdf(model_name, pose, topic, width, height, hfov):
    x, y, z, roll, pitch, yaw = pose
    return (
        '<?xml version="1.0"?>\n'
        '<sdf version="1.9">\n'
        f' <model name="{model_name}">\n'
        '  <static>true</static>\n'
        '  <link name="cam_link">\n'
        f'   <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>\n'
        f'   <sensor name="{model_name}" type="camera">\n'
        '    <always_on>true</always_on>\n'
        '    <update_rate>30</update_rate>\n'
        f'    <topic>{topic}</topic>\n'
        '    <camera>\n'
        f'     <horizontal_fov>{hfov}</horizontal_fov>\n'
        f'     <image><width>{width}</width><height>{height}</height>'
        '<format>R8G8B8</format></image>\n'
        '     <clip><near>0.05</near><far>60</far></clip>\n'
        '    </camera>\n'
        '   </sensor>\n'
        '  </link>\n'
        ' </model>\n'
        '</sdf>\n'
    )


def gz_create(world, sdf_str, env):
    req = "sdf: " + json.dumps(sdf_str) + " allow_renaming: false"
    subprocess.run(
        [
            "gz", "service", "-s", f"/world/{world}/create",
            "--reqtype", "gz.msgs.EntityFactory", "--reptype", "gz.msgs.Boolean",
            "--timeout", "4000", "--req", req,
        ],
        check=True, env=env, capture_output=True,
    )


def gz_unpause(world, env):
    r = subprocess.run(
        [
            "gz", "service", "-s", f"/world/{world}/control",
            "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000", "--req", "pause: false",
        ],
        env=env, capture_output=True, text=True,
    )
    return r.returncode == 0 and "data: true" in r.stdout


def wait_and_unpause(world, env, timeout=90.0):
    import time

    t0 = time.time()
    while time.time() - t0 < timeout:
        if gz_unpause(world, env):
            return True
        time.sleep(1.0)
    raise TimeoutError(f"/world/{world}/control not reachable within {timeout}s")


def spawn_shot_cameras(world, cameras, env, width, height, hfov=1.2):
    topics = {}
    for name, pose in cameras.items():
        topic = f"/shot/{name}/image"
        gz_create(world, camera_sdf(name, pose, topic, width, height, hfov), env)
        topics[name] = topic
    return topics


def spawn_tracked_bag(world, model_sdf, env, color=(0.95, 0.45, 0.10),
                      y=-1.60, yaw=1.5707963267948966, name="tracked_bag",
                      use_mesh=False):
    """Spawn the tracked bag. use_mesh=True keeps the DAE visual (colours fixed);
    otherwise a solid coloured box visual is used for maximum legibility."""
    import xml.etree.ElementTree as ET

    root = ET.parse(model_sdf).getroot()
    m = root.find("model")
    m.set("name", name)
    size = [float(x) for x in m.findtext(".//collision/geometry/box/size").split()]
    z = 0.300 + size[2] / 2 + 0.002
    ET.SubElement(m, "pose").text = f"1.3 {y} {z} 0 0 {yaw}"
    pl = ET.SubElement(m, "plugin", filename="gz-sim-pose-publisher-system",
                       name="gz::sim::systems::PosePublisher")
    for k, v in [("topic", "/model/%s/pose" % name), ("publish_model_pose", "true"),
                 ("publish_link_pose", "false"), ("publish_visual_pose", "false"),
                 ("publish_collision_pose", "false"), ("use_pose_vector_msg", "true"),
                 ("update_frequency", "50")]:
        ET.SubElement(pl, k).text = v
    for uri in m.findall(".//mesh/uri"):
        if uri.text.startswith("model://"):
            uri.text = (Path(model_sdf).parent.parent / uri.text[8:]).resolve().as_uri()
    vis = m.find(".//visual")
    if not use_mesh:
        geo = vis.find("geometry")
        if geo is None:
            geo = ET.SubElement(vis, "geometry")
        for ch in list(geo):
            geo.remove(ch)
        box = ET.SubElement(geo, "box")
        ET.SubElement(box, "size").text = f"{size[0]} {size[1]} {size[2]}"
    if not use_mesh:
        mat = vis.find("material")
        if mat is None:
            mat = ET.SubElement(vis, "material")
        for tag in ("ambient", "diffuse"):
            el = mat.find(tag)
            if el is None:
                el = ET.SubElement(mat, tag)
            el.text = f"{color[0]} {color[1]} {color[2]} 1"
    elif vis.find("material") is not None:
        vis.remove(vis.find("material"))
    gz_create(world, ET.tostring(root, encoding="unicode"), env)
    return size


def build_capture_node(cameras, outdir, fps):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Empty, Float64MultiArray
    from tf2_msgs.msg import TFMessage
    from cv_bridge import CvBridge
    import cv2

    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    class CaptureNode(Node):
        def __init__(self):
            super().__init__("storyboard_capture")
            self.bridge = CvBridge()
            self.outdir = outdir
            self.fps = fps
            self.recording = False
            self.writers = {}
            self.frames = {name: 0 for name in cameras}
            self.status = {}
            self.bag = None
            self.bag_at = -1e9
            self.create_subscription(String, "/aircraft/status", self._on_status, 10)
            self.create_subscription(TFMessage, "/aircraft/world_poses", self._on_poses, 10)
            self.pub_transfer = self.create_publisher(Empty, "/aircraft/start_transfer", 10)
            self.pub_entry = self.create_publisher(Empty, "/aircraft/start_entry", 10)
            self.pub_target = self.create_publisher(
                Float64MultiArray, "/aircraft/pose_target", 10
            )
            for name in cameras:
                self.create_subscription(
                    Image, f"/shot/{name}/image", self._mk_cb(name), 10
                )

        def _mk_cb(self, name):
            def cb(msg):
                if not self.recording:
                    return
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                w = self.writers.get(name)
                if w is None:
                    h, wd = frame.shape[:2]
                    w = cv2.VideoWriter(
                        str(self.outdir / f"{name}.mp4"),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        self.fps, (wd, h),
                    )
                    self.writers[name] = w
                w.write(frame)
                self.frames[name] += 1
            return cb

        def _on_status(self, msg):
            try:
                self.status = json.loads(msg.data)
            except Exception:
                pass

        def _on_poses(self, msg):
            for tr in msg.transforms:
                if "tracked_bag" in tr.child_frame_id:
                    p = tr.transform.translation
                    self.bag = (p.x, p.y, p.z)
                    self.bag_at = self.get_clock().now().nanoseconds * 1e-9

        def start_recording(self):
            self.recording = True

        def stop_recording(self):
            self.recording = False
            for w in self.writers.values():
                w.release()
            return dict(self.frames)

    return CaptureNode()


def run_bg(cmd, env, log):
    return subprocess.Popen(
        cmd, env=env, stdout=open(log, "w"), stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop_proc(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), 15)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except Exception:
            proc.kill()


def kill_gazebo():
    subprocess.run(["pkill", "-9", "-f", "[g]z sim"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "[p]arameter_bridge"], capture_output=True)
    subprocess.run(["pkill", "-9", "-f", "[p]arameter_bridge"], capture_output=True)
