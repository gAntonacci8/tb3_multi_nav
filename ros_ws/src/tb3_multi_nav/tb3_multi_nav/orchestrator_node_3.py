    


import math, random, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger, Empty

def q_yaw(y):
    q = Quaternion(); q.z = math.sin(y/2); q.w = math.cos(y/2); return q

def dist_xy(p1, p2): return math.hypot(p1.x - p2.x, p1.y - p2.y)

class Orchestrator(Node):
    def __init__(self):
        super().__init__('tag_orchestrator')

        self.tag_threshold = 1.0
        self.zone_r = 1.0
        self.start = {'robot1': (-3.0, -1.0, 0.0), 'robot2': (-3.0,  1.0, 0.0)} #CHECK maybe put it to none, didn't test it yet
        self.safe  = {'robot1': (-1.0, -1.0, 0.0), 'robot2': (-1.0,  1.0, 0.0)}
        self.safe_zones = [
            ( 4.0,  1.0, 0.0),
            ( 4.0, -1.0, 0.0),
            (-3.5,  3.7, 0.0),
            (-3.5, -3.7, 0.0),
        ]
        # Scoreboard
        self.score = {'robot1': 0, 'robot2': 0}

        # Evader/chaser choice
        self.evader = random.choice(['robot1','robot2'])
        self.chaser = 'robot2' if self.evader=='robot1' else 'robot1'
        self.get_logger().info(f'Roles: evader={self.evader}, chaser={self.chaser}')

        self.pose = {'robot1': None, 'robot2': None}
        self.create_subscription(PoseStamped, '/robot1/target_pose', self._on_pose1, 10)
        self.create_subscription(PoseStamped, '/robot2/target_pose', self._on_pose2, 10)

        self.ac = {
            'robot1': ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose'),
            'robot2': ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')
        }
        self.gh = {'robot1': None, 'robot2': None} #goal handles

        self.state = 'INIT'
        self.evader_goal_sent = False
        self.round_auto_restart = True
        self.round_restart_delay = 5.0
        self.start_tol = 0.3

        # time handlers #To try to stop it from freezing
        self._check_start_timer = None
        self._auto_restart_timer = None

        # send chaser goal only when target moves > min_delta
        self._last_chaser_target = None
        self.chase_min_delta = 0.25  # m

        # clean costmap services To try to stop it from freezing
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
        # ### CHANGE: evader watchdog – if evader freezes, happens sometimes
        self.create_timer(1.0, self._evader_watchdog)

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
        self.get_logger().info(f'Round START → evader={self.evader}, chaser={self.chaser}')

        self._pick_safe_zone_for_evader()
        self.evader_goal_sent = False
        self._last_chaser_target = None   #: reset chaser cache
        self.state = 'RUNNING'
        self._send_evader_safe()

    def _pick_safe_zone_for_evader(self):
        x,y,yaw = random.choice(self.safe_zones)
        self.safe[self.evader] = (x,y,yaw)
        self.get_logger().info(f'Picked SAFE ZONE for {self.evader}: ({x:.1f},{y:.1f})')

    def _send_evader_safe(self):
        if self.evader_goal_sent or self.state!='RUNNING': return
        x,y,yaw = self.safe[self.evader]
        self.get_logger().info(f'Evader {self.evader} → SAFE ({x:.1f},{y:.1f})')
        self._send_goal(self.evader, x, y, yaw)
        self.evader_goal_sent = True

    def _tick_chaser(self):
        if self.state!='RUNNING': return
        p = self.pose[self.evader]
        if p is None: return

        # ### CHANGE: ne cancel-uj stalno; preempt radi automatski; pošalji samo kad ima smisla
        if self._last_chaser_target is not None:
            if dist_xy(p, self._last_chaser_target) < self.chase_min_delta:
                return  # target still inside min_delta
        self._last_chaser_target = type('P', (), {'x': p.x, 'y': p.y})()

        self._send_goal(self.chaser, p.x, p.y, 0.0)
    
     # ### SCOREBOARD: helper
    def _award_point(self, winner: str, reason: str):
        self.score[winner] += 1
        s1, s2 = self.score['robot1'], self.score['robot2']
        self.get_logger().info(
            f'POINT → {winner} ({reason}). SCORE: robot1 {s1} - {s2} robot2'
        )

    def _tick_events(self):
        if self.state != 'RUNNING':
            return
        if self.pose[self.evader] is None or self.pose[self.chaser] is None:
            return

        # TAG?
        distance = dist_xy(self.pose[self.evader], self.pose[self.chaser])
        if distance <= self.tag_threshold:
            # ### SCOREBOARD: chaser point!
            self._award_point(self.chaser, 'tag')
            self.get_logger().info(f'TAG DETECTED! Distance: {distance:.2f}m')
            self._reset_round("tag")
            return

        # SAFE ZONE?
        evader_pos = self.pose[self.evader]
        safe_x, safe_y, _ = self.safe[self.evader]
        safe_distance = math.hypot(evader_pos.x - safe_x, evader_pos.y - safe_y)
        if safe_distance <= self.zone_r:
            # ### SCOREBOARD: evader point!
            self._award_point(self.evader, 'safe_zone')
            self.get_logger().info(f'EVADER REACHED SAFE ZONE! Distance: {safe_distance:.2f}m')
            self._reset_round("safe_zone")

    def _evader_watchdog(self):
        # if RUNNING and evader has no active goal → resend safe goal
        if self.state!='RUNNING': return
        if not self.evader_goal_sent: return
        if self.gh[self.evader] is None:
            self.get_logger().warn('Evader goal missing → resend SAFE')
            self.evader_goal_sent = False
            self._send_evader_safe()

    def _reset_round(self, reason):
        self.get_logger().info(f'Resetting round: {reason}')
        self.state = 'RETURNING'

        # cancel both
        for r in ['robot1','robot2']:
            self._cancel(r)

        # clean costmaps before new planning #To try to stop it from freezing
        self._clear_costmaps('robot1')
        self._clear_costmaps('robot2')

        # return to start
        for r in ['robot1','robot2']:
            x,y,yaw = self.start[r]
            self._send_goal(r, x, y, yaw)

        # CHANGE: cancel old timer and make new to follow "both at start"
        if self._check_start_timer:
            self._check_start_timer.cancel()
        self._check_start_timer = self.create_timer(0.5, self._check_both_at_start)

        # block evader to run before start
        self.evader_goal_sent = True
        self._last_chaser_target = None

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

        self.get_logger().info('Both at start → IDLE')
        self.state = 'IDLE'
        if self._check_start_timer:         
            self._check_start_timer.cancel(); self._check_start_timer = None

        # I might have made a mess here with timers CHECK!
        if self.round_auto_restart:
            if self._auto_restart_timer:
                self._auto_restart_timer.cancel()
            self._auto_restart_timer = self.create_timer(
                self.round_restart_delay, lambda: self.start_round(swap_roles=True)
            )
        else:
            self.get_logger().info('Waiting for manual start')

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
            _ = future.result()  # u Jazzy: ima status/result; nije nam bitno ovdje
            # ako chaser završi (stigne na staru target tačku), sledeći tick će mu dati novu
            # ako evader završi prije reset eventa, watchdog će ga držati živim
        except Exception as e:
            self.get_logger().error(f'Error in goal result for {robot}: {e}')
        finally:
            # ### CHANGE: goal više nije aktivan
            self.gh[robot] = None

    def _cancel(self, robot):
        if self.gh[robot] is not None:
            try:
                self.gh[robot].cancel_goal_async()
            except Exception as e:
                self.get_logger().error(f'Error canceling goal for {robot}: {e}')
            finally:
                self.gh[robot] = None

    def _clear_costmaps(self, robot):
        # ### CHANGE: helper – clears costmap #doesnt seem to work TEST!
        for key in ('g','l'):
            cli = self._clear_clients[robot][key]
            if not cli.service_is_ready(): continue
            try:
                cli.call_async(Empty.Request())
            except Exception:
                pass

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
