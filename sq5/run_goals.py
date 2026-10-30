#!/usr/bin/env python3
"""SQ5: sends a list of goals to a running EasyNav, one after the other, and records per goal:
outcome, time, distance travelled and final error (true pose from Gazebo), the closest laser
return, the localization error (EasyNav's map -> base_footprint against the true pose) and the
CPU of EasyNav's process. A goal that fails or times out is cancelled; the next one starts from
wherever the robot is.

Usage: run_goals.py goals.json out.json [--timeout 120] [--model kobuki] [--world default]
                    [--settle 10] [--trace trace.csv] [--episode K]
Goals may have a z (3D worlds): it is sent with the goal, and the final error reports it apart.
Gazebo's z is turned into the map's with the start: the map has the spec's start_z under the robot.
--trace: every sample (sim time, goal, true, estimated and odometry poses, closest return).
--episode K: one independent episode, the leg to goal K alone, in a freshly launched simulation:
the robot is placed at the pose of goal K-1 (Gazebo's set_pose; the spawn for K = 0), the
localizer is reset there through /initialpose, and goal K is sent after the settle time. The
episode is 'setup_failed' if the robot is not where it was placed.
"""
import argparse
import json
import math
import re
import subprocess
import time

from easynav_goalmanager_py.goal_manager_client import ClientState, GoalManagerClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import psutil
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

DONE = {ClientState.NAVIGATION_FINISHED: 'finished', ClientState.NAVIGATION_FAILED: 'failed',
        ClientState.NAVIGATION_REJECTED: 'rejected', ClientState.ERROR: 'error',
        ClientState.NAVIGATION_CANCELLED: 'cancelled'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def model_index(model, world='default', tries=30):
    """Place of the model among the poses Gazebo publishes (same order in every message)."""
    for _ in range(tries):
        try:
            out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                                 capture_output=True, text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            out = ''
        names = re.findall(r'^pose \{\n  name: "([^"]*)"', out, flags=re.M)
        if model in names:
            return names.index(model)
        time.sleep(1.0)
    raise SystemExit(f'model {model} not found in /world/{world}/pose/info')


def model_pose_cli(model, tries=5):
    """The model's pose (x, y, z, yaw) from `gz model -p`, which names it; None if unavailable."""
    for _ in range(tries):
        try:
            out = subprocess.run(['gz', 'model', '-m', model, '-p'], capture_output=True,
                                 text=True, timeout=10).stdout
        except subprocess.TimeoutExpired:
            out = ''
        m = re.search(r'Pose \[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*\[([^\]]+)\]\s*\[([^\]]+)\]', out)
        if m:
            x, y, z = (float(v) for v in m.group(1).split())
            yaw = float(m.group(2).split()[2])
            return x, y, z, yaw
        time.sleep(1.0)
    return None


class Runner(Node):

    def __init__(self, model, world, scan):
        super().__init__('sq5_runner', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.model = model
        # The bridge drops the entities' names and Gazebo does not keep their order from one
        # message to the next: the model's place in the first message is only a first guess.
        # The entities at the robot's start are candidates (the model, its links, the ground
        # plane...); once the odometry says the robot has moved, the candidate that moved the
        # same distance is the robot, and it is kept.
        try:
            self.index = model_index(model, world)
        except SystemExit as e:
            print(f'warning: {e}; the true pose is found by its start', flush=True)
            self.index = None
        self.start = None          # set by main from the goals' spec
        self.candidates = None     # [(index, (x, y))] at the start
        self.locked = False
        self.odom0 = None
        self.xy0 = None            # every entity's place when odom0 was taken
        self.before = None         # every entity's place before a teleport (episodes)
        self.xy = None             # the last message's places
        self.xyz = None
        self.client = GoalManagerClient(self)
        self.gt = None
        self.gt_z = 0.0
        self.min_range = math.inf
        self.last_range = math.nan
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(TFMessage, '/gt_pose', self.on_gt, 10)
        if scan:
            self.create_subscription(LaserScan, scan, self.on_scan, 10)

    def lock_by_pose(self, pose):
        """The entity at a pose known to be the robot's (from the CLI) is the robot."""
        if self.xyz is None:
            return False
        i = min(range(len(self.xyz)), key=lambda k: math.dist(self.xyz[k], pose[:3]))
        if math.dist(self.xyz[i], pose[:3]) > 0.3:
            return False
        self.index, self.locked = i, True
        return True

    def on_gt(self, msg):
        xy = [(t.transform.translation.x, t.transform.translation.y) for t in msg.transforms]
        self.xy = xy
        self.xyz = [(t.transform.translation.x, t.transform.translation.y,
                     t.transform.translation.z) for t in msg.transforms]
        if not self.locked and self.candidates is None and self.start is not None:
            # After a teleport, an entity that was already there is scenery, not the robot
            self.candidates = [(i, p) for i, p in enumerate(xy)
                               if math.dist(p, self.start) < 1.5
                               and not (self.before is not None and i < len(self.before)
                                        and math.dist(self.before[i], p) < 0.5)]
            if self.candidates and self.index not in [i for i, _ in self.candidates]:
                self.index = self.candidates[0][0]
        if not self.locked and self.candidates:
            o = self.pose('odom')
            if o is not None:
                if self.odom0 is None:
                    self.odom0, self.xy0 = o, xy
                moved = math.dist(o[:2], self.odom0[:2])
                if moved > 1.0:  # the entity that moved as much as the odometry is the robot
                    disp = [math.dist(xy[i], self.xy0[i]) if i < len(self.xy0) else 0.0
                            for i in range(len(xy))]
                    best = min(range(len(xy)), key=lambda i: abs(disp[i] - moved))
                    if disp[best] > 0.5 * moved:
                        if best != self.index:
                            print(f'true pose: entity {best} instead of {self.index}', flush=True)
                        self.index = best
                        self.locked = True
        if self.index is None or self.index >= len(msg.transforms):
            return
        t = msg.transforms[self.index]
        self.gt = (t.transform.translation.x, t.transform.translation.y,
                   yaw_of(t.transform.rotation))
        self.gt_z = t.transform.translation.z

    def on_scan(self, msg):
        r = [x for x in msg.ranges if msg.range_min < x < msg.range_max]
        if r:
            self.last_range = min(r)
            self.min_range = min(self.min_range, self.last_range)

    def estimate(self):
        p = self.pose('map')
        return p[:2] if p else None

    def pose(self, frame, z=False):
        try:
            t = self.tf.lookup_transform(frame, 'base_footprint', Time())
            p = (t.transform.translation.x, t.transform.translation.y,
                 yaw_of(t.transform.rotation))
            return p + (t.transform.translation.z,) if z else p
        except Exception:
            return None


def set_pose(world, model, x, y, z, yaw):
    """Teleports the model with Gazebo's set_pose service; returns whether it was accepted."""
    req = (f'name: "{model}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    r = subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype',
                        'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                        '--req', req], capture_output=True, text=True)
    return 'data: true' in r.stdout


def easynav_process():
    """EasyNav's process: the system_main executable (not a shell or launch mentioning it)."""
    for p in psutil.process_iter(['name', 'exe']):
        if p.info['name'] == 'system_main' or (p.info['exe'] or '').endswith('/system_main'):
            return p
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('goals')
    ap.add_argument('out')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--model', default='kobuki')
    ap.add_argument('--world', default='default')
    ap.add_argument('--scan', default='/scan_raw', help="LaserScan for the closest return ('': none)")
    ap.add_argument('--settle', type=float, default=10.0)
    ap.add_argument('--trace')
    ap.add_argument('--episode', type=int, default=None, help='the leg to this goal alone')
    ap.add_argument('--episode-from', type=int, default=None,
                    help='goal where the episode starts (default: the goal before)')
    ap.add_argument('--warmup', action='store_true',
                    help='before the first goal, a goal at the robot\'s own pose: the first goal of '
                         'a session carries the stack\'s one-off initialisation (6-9 s in the NavMap '
                         'stack), which an episode must not measure')
    a = ap.parse_args()
    spec = json.load(open(a.goals))
    if a.episode is not None and not 0 <= a.episode < len(spec['goals']):
        raise SystemExit(f'no goal {a.episode}')
    rclpy.init()
    n = Runner(a.model, a.world, a.scan)
    n.start = spec.get('start', [0.0, 0.0])
    t_end = time.time() + 60
    while time.time() < t_end and (not n.client.wait_for_server(timeout_sec=1.0) or n.gt is None):
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.gt is None:
        raise SystemExit('no EasyNav or no true pose')
    start = spec.get('start', [0.0, 0.0])
    # Gazebo's z in the map frame: the robot, landed from its spawn, stands at start_z. The
    # pose comes from the CLI, which names the model (the bridge does not); wait for the
    # landing (z steady for 1 s, 20 s at most): a spawn is some metres above the ground
    def landed_pose(near=None):
        t_end, prev = time.time() + 20.0, None
        while time.time() < t_end:
            t_next = time.time() + 1.0
            while time.time() < t_next:
                rclpy.spin_once(n, timeout_sec=0.1)
            cur = model_pose_cli(a.model)
            if cur is None:
                continue
            if near is not None and math.hypot(cur[0] - near[0], cur[1] - near[1]) > 1.5:
                prev = cur
                continue
            if prev is not None and abs(cur[2] - prev[2]) < 0.005:
                return cur
            prev = cur
        return None
    cli = landed_pose()
    if cli is None:
        raise SystemExit('no pose of the model from the Gazebo CLI')
    if not n.lock_by_pose(cli):
        print(f'warning: no entity at the CLI pose {cli}; the true pose is found by its start', flush=True)
    z_off = spec.get('start_z', 0.0) - cli[2]
    if math.hypot(cli[0] - start[0], cli[1] - start[1]) > 0.5:
        raise SystemExit(f'the model at {cli} is not at its start {start}')
    episode_ok = True
    src = None
    if a.episode:  # the leg to goal K starts at goal K-1: place the robot and the localizer there
        src = a.episode_from if a.episode_from is not None else a.episode - 1
        p = spec['goals'][src]
        z = p.get('z', spec.get('start_z', 0.0)) - z_off + 0.2
        n.before = n.xy  # the scenery stays where it is; the robot is what appears at p
        if not set_pose(a.world, a.model, p['x'], p['y'], z, p['yaw']):
            print('warning: set_pose not accepted', flush=True)
        # Land (on a slope the robot may slide after landing: up to 1.5 m) and find the robot
        # again among the entities, at the pose the CLI gives; the localizer is reset at it
        cli = landed_pose(near=(p['x'], p['y']))
        n.start, n.candidates, n.locked, n.odom0 = [p['x'], p['y']], None, False, None
        episode_ok = cli is not None and n.lock_by_pose(cli)
        if episode_ok:
            t_next = time.time() + 1.0
            while time.time() < t_next:
                rclpy.spin_once(n, timeout_sec=0.1)
            episode_ok = n.gt is not None and math.hypot(n.gt[0] - cli[0], n.gt[1] - cli[1]) < 0.3
        if not episode_ok:
            print(f'warning: CLI pose {cli}, true pose {n.gt} after placing the robot at {p}', flush=True)
        px, py, pyaw = (cli[0], cli[1], cli[3]) if episode_ok else (p['x'], p['y'], p['yaw'])
        init = PoseWithCovarianceStamped()
        init.header.frame_id = 'map'
        init.pose.pose.position.x, init.pose.pose.position.y = px, py
        init.pose.pose.position.z = p.get('z', 0.0)
        init.pose.pose.orientation.z = math.sin(pyaw / 2)
        init.pose.pose.orientation.w = math.cos(pyaw / 2)
        init.pose.covariance[0] = init.pose.covariance[7] = 0.05 ** 2
        init.pose.covariance[35] = 0.01 ** 2
        pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # The localizer may still be starting (the NavMap stack loads its map first): republish
        # the pose every second until its estimate is at the placed pose, 60 s at most
        t_end = time.time() + 60.0
        reset_s = None
        while time.time() < t_end:
            init.header.stamp = n.get_clock().now().to_msg()
            pub.publish(init)
            t_next = time.time() + 1.0
            while time.time() < t_next:
                rclpy.spin_once(n, timeout_sec=0.1)
            est = n.estimate()
            if est and math.hypot(est[0] - px, est[1] - py) < 0.3:
                reset_s = round(60.0 - (t_end - time.time()), 1)
                break
        if reset_s is None:
            print('warning: the localizer did not take the initial pose', flush=True)
            episode_ok = False
        print(f'episode {a.episode}: robot placed at goal {src} '
              f'({"ok" if episode_ok else "NOT where placed"}; localizer reset in {reset_s} s)',
              flush=True)
    t_end = time.time() + a.settle  # let the localizer settle
    while time.time() < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if a.warmup and episode_ok:
        est = n.pose('map', z=True)
        if est:
            w = PoseStamped()
            w.header.frame_id = 'map'
            w.header.stamp = n.get_clock().now().to_msg()
            w.pose.position.x, w.pose.position.y, w.pose.position.z = est[0], est[1], est[3]
            w.pose.orientation.z, w.pose.orientation.w = math.sin(est[2] / 2), math.cos(est[2] / 2)
            if n.client.get_state() in DONE:
                n.client.reset()
            t0 = time.time()
            n.client.send_goal(w)
            while time.time() - t0 < 40.0 and n.client.get_state() not in DONE:
                rclpy.spin_once(n, timeout_sec=0.05)
            st = n.client.get_state()
            if st not in DONE:
                n.client.cancel()
            print(f'warm-up goal at the robot\'s pose: {DONE.get(st, "not finished")} in '
                  f'{time.time() - t0:.1f} s', flush=True)
            t_end = time.time() + 3.0
            while time.time() < t_end:
                rclpy.spin_once(n, timeout_sec=0.1)
    proc = easynav_process()
    if proc is None:
        print('warning: EasyNav process not found, no CPU figures', flush=True)
    results = []
    trace = None
    if a.trace:
        trace = open(a.trace, 'w')
        trace.write('t,goal,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,odom_x,odom_y,odom_yaw,range,'
                    'gt_z,est_z\n')
    legs = [a.episode] if a.episode is not None else range(len(spec['goals']))
    for k in legs:
        g = spec['goals'][k]
        if not episode_ok:
            res = {'goal': k, 'x': g['x'], 'y': g['y'], 'outcome': 'setup_failed', 'time_s': 0.0,
                   'travelled_m': 0.0, 'final_error_m': None, 'final_dz_m': None,
                   'min_range_m': None, 'loc_error_mean_m': None, 'loc_error_max_m': None,
                   'cpu_percent': None, 'episode_from': src}
            results.append(res)
            print(json.dumps(res), flush=True)
            json.dump({'goals': spec, 'results': results}, open(a.out, 'w'), indent=1)
            continue
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = n.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = g['x'], g['y']
        goal.pose.position.z = g.get('z', 0.0)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(g['yaw'] / 2), math.cos(
            g['yaw'] / 2)
        if n.client.get_state() in DONE:
            n.client.reset()
        n.min_range = math.inf
        if proc:
            proc.cpu_percent(None)
            cpu0 = sum(proc.cpu_times()[:2])
        t0 = n.get_clock().now()
        w0 = time.time()
        last = n.gt
        last_z = n.gt_z
        travelled, loc_err = 0.0, []
        n.client.send_goal(goal)
        outcome = 'timeout'
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            if n.gt and last:
                travelled += math.dist((*n.gt[:2], n.gt_z), (*last[:2], last_z))
            last, last_z = n.gt, n.gt_z
            est = n.estimate()
            if est and n.gt:
                loc_err.append(math.hypot(est[0] - n.gt[0], est[1] - n.gt[1]))
            if trace and n.gt:
                e, o = n.pose('map', z=True), n.pose('odom')
                nan3 = (math.nan,) * 3
                row = (n.get_clock().now().nanoseconds * 1e-9, k, *n.gt, *(e[:3] if e else nan3),
                       *(o or nan3), n.last_range, n.gt_z + z_off, e[3] if e else math.nan)
                trace.write(','.join(f'{v:.4f}' if isinstance(v, float) else str(v)
                                     for v in row) + '\n')
            if proc is not None and not proc.is_running():
                raise SystemExit('EasyNav died')
            state = n.client.get_state()
            if state in DONE:
                outcome = DONE[state]
                break
            if (n.get_clock().now() - t0) > Duration(seconds=a.timeout):
                n.client.cancel()
                t_c = time.time() + 5.0  # wait for the cancellation to be confirmed
                while time.time() < t_c and n.client.get_state() not in DONE:
                    rclpy.spin_once(n, timeout_sec=0.05)
                break
        dt = (n.get_clock().now() - t0).nanoseconds * 1e-9
        wall = time.time() - w0
        cpu = (sum(proc.cpu_times()[:2]) - cpu0) / wall * 100 if proc else None
        err = math.hypot(n.gt[0] - g['x'], n.gt[1] - g['y'])
        res = {'goal': k, 'x': g['x'], 'y': g['y'], 'outcome': outcome, 'time_s': dt,
               'travelled_m': travelled, 'final_error_m': err,
               'final_dz_m': n.gt_z + z_off - g.get('z', n.gt_z + z_off),
               'min_range_m': None if math.isinf(n.min_range) else n.min_range,
               'loc_error_mean_m': sum(loc_err) / len(loc_err) if loc_err else None,
               'loc_error_max_m': max(loc_err) if loc_err else None, 'cpu_percent': cpu}
        if a.episode is not None:
            res['episode_from'] = src
        results.append(res)
        print(json.dumps(res), flush=True)
        json.dump({'goals': spec, 'results': results}, open(a.out, 'w'), indent=1)
    if trace:
        trace.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
