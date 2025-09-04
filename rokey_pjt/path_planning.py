#!/usr/bin/env python3
import math
import argparse
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath

from tf2_ros import Buffer, TransformListener

# ---------- helpers ----------
def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)  # rad

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))  # x,y,z,w

def wrap_pi(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
# --------------------------------

class PlanNode(Node):
    def __init__(self, ns, frame, base_frame, map_topic, publish_path_topic, publish_samples):
        super().__init__('path_planning_node')
        self.ns = ns
        self.frame = frame
        self.base_frame = base_frame

        # Action clients
        self.plan_cli = ActionClient(self, ComputePathToPose, f'{ns}/compute_path_to_pose')
        self.follow_cli = ActionClient(self, FollowPath, f'{ns}/follow_path')

        # TF buffer/listener
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # Optional: wait for map (latched-like QoS)
        self.map_ready = False
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

        # Optional publisher for resampled path
        self.publish_path_topic = publish_path_topic
        self.publish_samples = publish_samples
        self.path_pub = None
        if publish_path_topic:
            pub_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST
            )
            self.path_pub = self.create_publisher(Path, publish_path_topic, pub_qos)

    # ---------- readiness ----------
    def _on_map(self, _msg):
        self.map_ready = True

    def wait_for_tf(self, timeout_sec=5.0):
        # need transform from base_frame -> frame (e.g., base_link -> map)
        ok = self.tf_buf.can_transform(
            self.frame, self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=float(timeout_sec))
        )
        if not ok:
            self.get_logger().error(f'TF missing: {self.frame} <- {self.base_frame}')
        return ok

    def wait_for_map(self, timeout_sec=5.0):
        # Just best-effort; Nav2 내부에서는 map 유무를 자체 처리함.
        if self.map_ready:
            return True
        self.get_logger().info(f'Waiting for map on topic for up to {timeout_sec:.1f}s...')
        end = self.get_clock().now() + Duration(seconds=float(timeout_sec))
        while rclpy.ok() and self.get_clock().now() < end:
            if self.map_ready:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.map_ready:
            self.get_logger().warn('Map not received yet (continuing anyway).')
        return self.map_ready

    # ---------- core ----------
    def compute_path(self, x, y, yaw_deg, timeout=15.0):
        # Wait servers
        self.get_logger().info(f'Waiting for servers...')
        if not self.plan_cli.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('ComputePathToPose server not available.')
            return None
        # FollowPath는 옵션이라 필수 대기는 안 함

        # Readiness guards
        self.wait_for_map(3.0)  # optional
        if not self.wait_for_tf(5.0):
            return None

        # Build goal
        yaw = math.radians(float(yaw_deg))
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = self.frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(x)
        goal.goal.pose.position.y = float(y)
        goal.goal.pose.position.z = 0.0
        qx,qy,qz,qw = quat_from_yaw(yaw)
        goal.goal.pose.orientation.x = qx
        goal.goal.pose.orientation.y = qy
        goal.goal.pose.orientation.z = qz
        goal.goal.pose.orientation.w = qw
        goal.use_start = False

        # Send & wait
        self.get_logger().info('Computing path...')
        gh_fut = self.plan_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut, timeout_sec=timeout)
        gh = gh_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Goal was rejected by planner.')
            return None

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        res = res_fut.result()
        if not res:
            self.get_logger().error('No result from planner.')
            return None

        path = res.result.path
        if not path.poses:
            self.get_logger().warn('Planner returned empty path.')
            return None

        return path

    # ---------- sampling / resample ----------
    def print_samples(self, path: Path, stride=None, samples=None):
        poses = path.poses
        L = len(poses)
        if L == 0:
            print('Empty path'); return

        if stride and stride > 1:
            idxs = list(range(0, L, stride))
        elif samples and samples > 0:
            if samples == 1:
                idxs = [0]
            else:
                idxs = [ round(k*(L-1)/(samples-1)) for k in range(samples) ]
        else:
            idxs = [0, L-1]
        idxs = sorted(set(max(0, min(L-1, i)) for i in idxs))

        print(f'# path poses = {L}, printing {len(idxs)} pose(s):')
        for i in idxs:
            p = poses[i].pose
            yaw = yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            print(f'- i={i:>4}: x={p.position.x:.3f}, y={p.position.y:.3f}, yaw={math.degrees(yaw):.1f} deg')

    def resample_by_arclen(self, path: Path, N: int, keep_endpoints=True) -> Path:
        import numpy as np
        xs = [ps.pose.position.x for ps in path.poses]
        ys = [ps.pose.position.y for ps in path.poses]
        yaws = [yaw_from_quat(ps.pose.orientation.x, ps.pose.orientation.y,
                               ps.pose.orientation.z, ps.pose.orientation.w) for ps in path.poses]
        L = len(xs)
        if L == 0:
            return Path()
        if L == 1 or N <= 1:
            out = Path()
            out.header = path.header
            out.header.stamp = self.get_clock().now().to_msg()
            out.poses = [path.poses[0]] * max(1, N)
            return out

        xs, ys, yaws = map(lambda a: np.asarray(a, dtype=float), (xs, ys, yaws))
        dx = np.diff(xs); dy = np.diff(ys)
        seg = np.hypot(dx, dy)
        cum = np.concatenate(([0.0], np.cumsum(seg)))
        total = float(cum[-1])
        if total < 1e-9:
            out = Path()
            out.header = path.header
            out.header.stamp = self.get_clock().now().to_msg()
            out.poses = [path.poses[0]] * N
            return out

        if keep_endpoints:
            targets = np.linspace(0.0, total, N)
        else:
            targets = np.linspace(0.0, total, N+2)[1:-1]

        def interp_at(s):
            k = int(np.searchsorted(cum, s, side='right') - 1)
            k = max(0, min(k, len(seg)-1))
            seglen = seg[k]
            alpha = 0.0 if seglen < 1e-12 else (s - cum[k]) / seglen
            x = xs[k] + alpha*(xs[k+1]-xs[k])
            y = ys[k] + alpha*(ys[k+1]-ys[k])
            dyaw = wrap_pi(yaws[k+1] - yaws[k])
            yaw = wrap_pi(yaws[k] + alpha*dyaw)
            return float(x), float(y), float(yaw)

        out = Path()
        out.header = path.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.poses = []
        for s in targets:
            x, y, yaw = interp_at(float(s))
            qx,qy,qz,qw = quat_from_yaw(yaw)
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            out.poses.append(ps)
        return out

    # ---------- optional follow ----------
    def follow_path(self, path: Path, timeout=30.0, controller_id="", goal_checker_id="", progress_checker_id=""):
        if not self.follow_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowPath server not available.')
            return False
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = controller_id
        goal.goal_checker_id = goal_checker_id
        goal.progress_checker_id = progress_checker_id

        self.get_logger().info('Sending FollowPath goal...')
        gh_fut = self.follow_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gh_fut, timeout_sec=10.0)
        gh = gh_fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('FollowPath goal rejected.')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        res = res_fut.result()
        self.get_logger().info(f'FollowPath finished with status: {getattr(res, "status", "unknown")}')
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='/robot1')
    ap.add_argument('--frame', default='map', help='target planning frame (map/odom)')
    ap.add_argument('--base-frame', default='base_link', help='base_link or base_footprint (환경에 맞게)')
    ap.add_argument('--map-topic', default='/robot1/map', help='map OccupancyGrid topic (읽기 전용, 준비 체크용)')
    ap.add_argument('--x', type=float, required=True)
    ap.add_argument('--y', type=float, required=True)
    ap.add_argument('--yaw', type=float, default=0.0, help='deg')
    ap.add_argument('--stride', type=int, help='M개 중 1개 출력')
    ap.add_argument('--samples', type=int, help='균등 N개만 출력')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--publish-path', default='', help='리샘플 Path를 퍼블리시할 토픽(비우면 미사용)')
    ap.add_argument('--publish-samples', type=int, default=0, help='퍼블리시 시 균등 N개로 리샘플(0이면 원본 그대로)')
    ap.add_argument('--follow', action='store_true', help='리샘플(또는 원본) Path로 FollowPath 실행')
    args = ap.parse_args()

    rclpy.init()
    node = PlanNode(
        ns=args.ns,
        frame=args.frame,
        base_frame=args.base_frame,
        map_topic=args.map_topic,
        publish_path_topic=args.publish_path,
        publish_samples=args.publish_samples
    )
    try:
        path = node.compute_path(args.x, args.y, args.yaw, timeout=args.timeout)
        if path is None:
            node.get_logger().error('Planning failed.')
            return

        # 콘솔 샘플 출력
        node.print_samples(path, stride=args.stride, samples=args.samples)

        # 퍼블리시 / 리샘플 (옵션)
        to_pub = path
        if node.path_pub:
            if node.publish_samples and node.publish_samples > 0:
                to_pub = node.resample_by_arclen(path, node.publish_samples, keep_endpoints=True)
            node.path_pub.publish(to_pub)
            node.get_logger().info(f'Published Path to {node.publish_path_topic} '
                                   f'({len(to_pub.poses)} poses).')

        # 추종 (옵션)
        if args.follow:
            node.follow_path(to_pub, timeout=max(30.0, args.timeout))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
