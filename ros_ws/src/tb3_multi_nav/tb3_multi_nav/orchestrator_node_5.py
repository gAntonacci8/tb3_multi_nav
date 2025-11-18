import math, random, rclpy, csv, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Empty
from pathlib import Path

def q_yaw(y):
    q = Quaternion(); q.z = math.sin(y/2); q.w = math.cos(y/2); return q

def dist_xy(p1, p2): return math.hypot(p1.x - p2.x, p1.y - p2.y)

class Orchestrator(Node):
    def __init__(self):
        super().__init__('tag_orchestrator')

        self.tag_threshold = 1.0
        self.zone_r = 1.0
        self.start = {'robot1': (-3.0, -1.0, 0.0), 'robot2': (-3.0,  1.0, 0.0)}
        self.safe  = {'robot1': None, 'robot2': None} 
        self.safe_zones = [
            ( 4.0,  1.0, 0.0),
            ( 4.0, -1.0, 0.0),
            (-3.5,  3.7, 0.0),
            (-3.5, -3.7, 0.0),
        ]
        # Scoreboard
        self.score = {'robot1': 0, 'robot2': 0}
        self.tags  = {'robot1': 0, 'robot2': 0}   
        self.safes = {'robot1': 0, 'robot2': 0} 

        # Evader/chaser choice - first round
        self.evader = random.choice(['robot1','robot2'])
        self.chaser = 'robot2' if self.evader=='robot1' else 'robot1'
        self.get_logger().info(f'Roles: evader={self.evader}, chaser={self.chaser}')
        self.pose = {'robot1': None, 'robot2': None}

        # Orchestrator is subscribed to target_pose topics of positions of both robots
        self.create_subscription(PoseStamped, '/robot1/target_pose', self._on_pose1, 10)
        self.create_subscription(PoseStamped, '/robot2/target_pose', self._on_pose2, 10)

        # To robotX is assigned an action that will tell it to which pose to navigate
        self.ac = {
            'robot1': ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose'),
            'robot2': ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')
        }

        self.gh = {'robot1': None, 'robot2': None} #goal handles

        self.state = 'INIT'
        self.evader_goal_sent = False
        self.round_restart_delay = 5.0
        self.start_tol = 0.3

        # time handlers
        self._check_start_timer = None
        self._auto_restart_timer = None

        # send chaser goal only when target moves > min_delta
        self._last_chaser_target = None
        self.chase_min_delta = 0.25  # m

        # clean costmap services 
        self._clear_clients = {
            'robot1': {
                'g': self.create_client(Empty, '/robot1/global_costmap/clear_entirely_global_costmap'),
                'l': self.create_client(Empty, '/robot1/local_costmap/clear_entirely_local_costmap')
            },
            'robot2': {
                'g': self.create_client(Empty, '/robot2/global_costmap/clear_entirely_global_costmap'),
                'l': self.create_client(Empty, '/robot2/local_costmap/clear_entirely_local_costmap')
            }
        }

        self.create_timer(0.5, self._ensure_ready)
        self.create_timer(1.0, self._tick_chaser)
        self.create_timer(0.3, self._tick_events)
        self.create_timer(1.0, self._evader_watchdog)
        self.create_timer(1.0, self._log_tick)
        
        # for export and plotting

        self.plan_remaining = {'robot1': None, 'robot2': None}
        self.create_subscription(NavPath, '/robot1/plan', self._on_plan_robot1, 10)
        self.create_subscription(NavPath, '/robot2/plan', self._on_plan_robot2, 10)

        # metrics/logging
        self.episode_id = 0
        self.round_start_time_s = None
        self.log_dir = Path.home() / "ros_ws/tag_metrics" 
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._csv_file = None
        self._csv_writer = None
        # --- unified CSV across all rounds
        self.round_idx = 0  #round counter
        self.all_csv_path = self.log_dir / "all_runs.csv"
        self._all_csv_file = None
        self._all_csv_writer = None
        
        new_file = not self.all_csv_path.exists()
        self._all_csv_file = open(self.all_csv_path, "a", newline="")
        self._all_csv_writer = csv.writer(self._all_csv_file)
        if new_file:
            self._all_csv_writer.writerow([
                "round",             # round index
                "t_s",               # seconds from round start
                "evader","chaser",
                "safe_x","safe_y",
                "dist_evader_to_safe",
                "path_remaining_evader",
                "dist_chaser_to_evader",
                "path_remaining_chaser",
                "goal_active_evader","goal_active_chaser",
                "outcome",           # "", "tag", "safe_zone"
                "winner",            # "" or robot id
                "robot1_score","robot2_score",
                "tags_robot1",
                "tags_robot2",
                "safes_robot1",
                "safes_robot2"
            ])
            self._all_csv_file.flush()

        self.get_logger().info("Orchestrator initialized")

    def _on_pose1(self, msg): self.pose['robot1'] = msg.pose.position
    def _on_pose2(self, msg): self.pose['robot2'] = msg.pose.position

    def _ensure_ready(self):
        if self.state != 'INIT': return
        servers_ready = all(self.ac[r].wait_for_server(timeout_sec=0.1) for r in ['robot1','robot2'])
        poses_ready   = all(self.pose[r] is not None for r in ['robot1','robot2'])
        if servers_ready and poses_ready:
            self.get_logger().info("All systems ready - starting first round")
            self.start_round(swap_roles=False)

    def start_round(self, swap_roles=True):
        if self.state not in ('INIT','IDLE'):
            self.get_logger().debug(f'Ignoring start_round in state={self.state}')
            return
        if swap_roles:
            self.evader, self.chaser = self.chaser, self.evader

        self.get_logger().info(f'Round START, evader={self.evader}, chaser={self.chaser}')

        self._pick_safe_zone_for_evader()
        self.evader_goal_sent = False
        self._last_chaser_target = None
        self.state = 'RUNNING'
        self._send_evader_safe()

        self._start_logging()

    def _pick_safe_zone_for_evader(self):
        x,y,yaw = random.choice(self.safe_zones)
        self.safe[self.evader] = (x,y,yaw) 
        self.get_logger().info(f'Picked SAFE ZONE for {self.evader}: ({x:.1f},{y:.1f})')

    def _send_evader_safe(self):
        if self.evader_goal_sent or self.state!='RUNNING': return
        x,y,yaw = self.safe[self.evader]
        self.get_logger().info(f'Evader {self.evader} sent to SAFE ({x:.1f},{y:.1f})')
        self._send_goal(self.evader, x, y, yaw)
        self.evader_goal_sent = True

    def _tick_chaser(self):
        if self.state!='RUNNING': return
        p = self.pose[self.evader]
        if p is None: return

        if self._last_chaser_target is not None:
            if dist_xy(p, self._last_chaser_target) < self.chase_min_delta:
                return  # target still inside min_delta
        self._last_chaser_target = p
        self._send_goal(self.chaser, p.x, p.y, 0.0)

     ### SCOREBOARD:
    def _award_point(self, winner: str, reason: str):
        self.score[winner] += 1
        s1, s2 = self.score['robot1'], self.score['robot2']
        self.get_logger().info(
            f'POINT to {winner} ({reason}). SCORE: robot1 {s1} - {s2} robot2'
        )

    def _tick_events(self):
        if self.state != 'RUNNING':
            return
        if self.pose[self.evader] is None or self.pose[self.chaser] is None:
            return

        # TAG?
        distance = dist_xy(self.pose[self.evader], self.pose[self.chaser])
        if distance <= self.tag_threshold:
            ### SCOREBOARD: chaser point!
            self.tags[self.chaser] += 1
            self._award_point(self.chaser, 'tag')
            self.get_logger().info(f'TAG DETECTED! Distance: {distance:.2f}m')
            self._reset_round("tag")
            return

        # SAFE ZONE?
        evader_pos = self.pose[self.evader]
        safe_x, safe_y, _ = self.safe[self.evader]
        safe_distance = math.hypot(evader_pos.x - safe_x, evader_pos.y - safe_y)
        if safe_distance <= self.zone_r:
            ### SCOREBOARD: evader point!
            self.safes[self.evader] += 1
            self._award_point(self.evader, 'safe_zone')
            self.get_logger().info(f'EVADER REACHED SAFE ZONE! Distance: {safe_distance:.2f}m')
            self._reset_round("safe_zone")

    def _evader_watchdog(self):
        # if RUNNING and evader has no active goal  resend safe goal
        if self.state!='RUNNING': return
        if not self.evader_goal_sent: return
        if self.gh[self.evader] is None:
            self.get_logger().warn('Evader goal missing, resend SAFE')
            self.evader_goal_sent = False
            self._send_evader_safe()

    def _reset_round(self, reason):
        self.round_idx += 1
        self.get_logger().info(f'Resetting round: {reason}')
        self.state = 'RETURNING'

        # clean costmaps before new planning
        self._clear_costmaps('robot1')
        self._clear_costmaps('robot2')

        # return to start
        for r in ['robot1','robot2']:
            x,y,yaw = self.start[r]
            self._send_goal(r, x, y, yaw)

        # cancel old timer and make new to follow "both at start"
        if self._check_start_timer:
            self._check_start_timer.cancel()
        self._check_start_timer = self.create_timer(0.5, self._check_both_at_start)

        # block evader to run before start
        self.evader_goal_sent = True
        self._last_chaser_target = None

        self._stop_logging(reason)

    def _check_both_at_start(self):
        if self.state!='RETURNING': return
        ok = True
        for r in ['robot1','robot2']:
            p = self.pose[r]
            if p is None: ok=False; break
            x,y,_ = self.start[r]
            if math.hypot(p.x - x, p.y - y) > self.start_tol:
                ok=False; break
        if not ok: return

        self.get_logger().info('Both at start, go to IDLE')
        self.state = 'IDLE'
        if self._check_start_timer:
            self._check_start_timer.cancel(); self._check_start_timer = None

        if self._auto_restart_timer:
            self._auto_restart_timer.cancel()
        self._auto_restart_timer = self.create_timer(
            self.round_restart_delay, self.start_round)

    def _send_goal(self, robot, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id='map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = q_yaw(yaw)
        fut = self.ac[robot].send_goal_async(goal)
        fut.add_done_callback(lambda f, r=robot: self._on_goal_response(r, f))

    def _on_goal_response(self, robot, future):
        try:
            gh = future.result()
            if not gh or not gh.accepted:
                self.get_logger().warn(f'Goal rejected for {robot}')
                return
            self.gh[robot] = gh
            gh.get_result_async().add_done_callback(lambda f, r=robot: self._on_goal_result(r, f))
        except Exception as e:
            self.get_logger().error(f'Error in goal response for {robot}: {e}')

    def _on_goal_result(self, robot, future):
        try:
            _ = future.result()  
        except Exception as e:
            self.get_logger().error(f'Error in goal result for {robot}: {e}')
        finally:
            self.gh[robot] = None

    def _clear_costmaps(self, robot):
        for key in ('g','l'):
            cli = self._clear_clients[robot][key]
            if not cli.service_is_ready(): continue
            try:
                cli.call_async(Empty.Request())
            except Exception:
                pass
                
    def _start_logging(self):
        """Open per-episode CSV and write header."""
        self.episode_id += 1
        self.round_start_time_s = time.time()
        fname = self.log_dir / f"episode_{self.episode_id:03d}.csv"
        self._csv_file = open(fname, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "t_s",
            "evader", "chaser",
            "safe_x", "safe_y",
            "dist_evader_to_safe",
            "path_remaining_evader",
            "dist_chaser_to_evader",
            "path_remaining_chaser",
            "goal_active_evader", "goal_active_chaser",
            "event",            # "", "tag", "safe_zone"
            "winner",           
            "score_robot1", "score_robot2",
            "tags_robot1", "tags_robot2",
            "safes_robot1", "safes_robot2"
        ])
        self._csv_file.flush()
        self.get_logger().info(f"Logging to {fname}")

    def _append_log_row(self, event_label=""):
        if self._csv_writer is None or self.round_start_time_s is None:
            return
        t = time.time() - self.round_start_time_s

        ev = self.evader
        ch = self.chaser
        ev_p = self.pose.get(ev)
        ch_p = self.pose.get(ch)

        # safe point (x,y)
        sx, sy, _ = self.safe[self.evader]

        # distances
        dist_e_safe = None
        if ev_p is not None:
            dist_e_safe = math.hypot(ev_p.x - sx, ev_p.y - sy)
        dist_c_e = None
        if ev_p is not None and ch_p is not None:
            dist_c_e = math.hypot(ch_p.x - ev_p.x, ch_p.y - ev_p.y)

        pr_e = self.plan_remaining.get(self.evader)
        pr_c = self.plan_remaining.get(self.chaser)

        self._csv_writer.writerow([
            f"{t:.2f}",
            ev, ch,
            f"{sx:.3f}", f"{sy:.3f}",
            f"{dist_e_safe:.3f}" if dist_e_safe is not None else "",
            f"{pr_e:.3f}"        if pr_e is not None        else "",
            f"{dist_c_e:.3f}"    if dist_c_e is not None    else "",
            f"{pr_c:.3f}"        if pr_c is not None        else "",
            1 if self.gh[ev] is not None else 0,
            1 if self.gh[ch] is not None else 0,
            "",                  # event 
            "",                  # winner
            self.score['robot1'], self.score['robot2'],
            self.tags['robot1'],  self.tags['robot2'],
            self.safes['robot1'], self.safes['robot2'],
        ])
        self._csv_file.flush()

    def _stop_logging(self, event_label):
        try:
            winner = self.chaser if event_label == "tag" else self.evader
            t = time.time() - self.round_start_time_s if self.round_start_time_s else ""

            ev = self.evader
            ch = self.chaser
            ev_p = self.pose.get(ev)
            ch_p = self.pose.get(ch)
            sx, sy, _ = self.safe[self.evader]

            dist_e_safe = ""
            if ev_p is not None:
                dist_e_safe = f"{math.hypot(ev_p.x - sx, ev_p.y - sy):.3f}"

            dist_c_e = ""
            if ev_p is not None and ch_p is not None:
                dist_c_e = f"{math.hypot(ch_p.x - ev_p.x, ch_p.y - ev_p.y):.3f}"

            pr_e = self.plan_remaining.get(ev)
            pr_c = self.plan_remaining.get(ch)

            #per-episode CSV (episode_XX.csv)
            if self._csv_writer is not None:
                self._csv_writer.writerow([
                    f"{t:.2f}" if t != "" else "",
                    ev, ch,
                    f"{sx:.3f}", f"{sy:.3f}",
                    dist_e_safe,
                    f"{pr_e:.3f}" if pr_e is not None else "",
                    dist_c_e,
                    f"{pr_c:.3f}" if pr_c is not None else "",
                    1 if self.gh.get(ev) else 0,
                    1 if self.gh.get(ch) else 0,
                    event_label,
                    winner,
                    self.score['robot1'], self.score['robot2'],
                    self.tags['robot1'],  self.tags['robot2'],
                    self.safes['robot1'], self.safes['robot2'],
                ])
                self._csv_file.flush()

            #all_runs.csv
            if self._all_csv_writer is not None:
                self._all_csv_writer.writerow([
                    self.round_idx,  # "round"
                    f"{t:.2f}" if t != "" else "",
                    ev, ch,
                    f"{sx:.3f}", f"{sy:.3f}",
                    dist_e_safe,
                    f"{pr_e:.3f}" if pr_e is not None else "",
                    dist_c_e,
                    f"{pr_c:.3f}" if pr_c is not None else "",
                    1 if self.gh.get(ev) else 0,
                    1 if self.gh.get(ch) else 0,
                    event_label,
                    winner,
                    self.score['robot1'],
                    self.score['robot2'],
                    self.tags['robot1'],
                    self.tags['robot2'],
                    self.safes['robot1'],
                    self.safes['robot2'],
                ])
                self._all_csv_file.flush()

        except Exception as _e:
            self.get_logger().warn(f"Error while stopping logging: {_e}")
        finally:
            if self._csv_file is not None:
                self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
            self.round_start_time_s = None

    def _log_tick(self):
        """Called at 1 Hz. Log only during RUNNING."""
        if self.state == 'RUNNING':
            self._append_log_row("")
    
    def _on_plan_robot1(self, msg: NavPath):
        self._update_plan_remaining('robot1', msg)

    def _on_plan_robot2(self, msg: NavPath):
        self._update_plan_remaining('robot2', msg)

    def _update_plan_remaining(self, robot: str, msg: NavPath):
        try:
            pts = msg.poses
            if not pts or len(pts) < 2:
                self.plan_remaining[robot] = 0.0
                return
            total = 0.0
            for i in range(1, len(pts)):
                p0 = pts[i-1].pose.position
                p1 = pts[i].pose.position
                total += math.hypot(p1.x - p0.x, p1.y - p0.y)
            self.plan_remaining[robot] = total
        except Exception:
            self.plan_remaining[robot] = None

    def destroy_node(self):
        try:
            if self._all_csv_file:
                self._all_csv_file.flush()
                self._all_csv_file.close()
        except Exception:
            pass
        return super().destroy_node()

def main():
    rclpy.init()
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
