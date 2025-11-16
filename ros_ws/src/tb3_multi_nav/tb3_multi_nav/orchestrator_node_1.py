import math, random, rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger

def q_yaw(y):
    q = Quaternion()
    q.z = math.sin(y/2)
    q.w = math.cos(y/2)
    return q

def dist(p1, p2): 
    return math.hypot(p1.x - p2.x, p1.y - p2.y)

class Orchestrator(Node):
    def __init__(self):
        super().__init__('tag_orchestrator')

        # --- params ---
        self.tag_threshold = 1.0  # tag distance
        self.zone_r = 1.0  # safe zone distance
        self.start = {
            'robot1': (-3.0, -1.0, 0.0),
            'robot2': (-3.0, 1.0, 0.0)
        }
        self.safe = {
            'robot1': (-1.0, -1.0, 0.0),    #CHECK  
            'robot2': (-1.0, 1.0, 0.0)    
        }

        # --- roles choice (first time random) ---
        self.evader = random.choice(['robot1', 'robot2'])
        self.chaser = 'robot2' if self.evader == 'robot1' else 'robot1'
        self.get_logger().info(f'Roles: evader={self.evader}, chaser={self.chaser}')

        # --- pose buffers ---
        self.pose = {'robot1': None, 'robot2': None}

        self.create_subscription(PoseStamped, '/robot1/target_pose', 
                            #    lambda m: self._on_pose('robot1', m),# function that is called when message arrives #CHECK1
                                self._on_pose1,
                                10) 
        self.create_subscription(PoseStamped, '/robot2/target_pose', 
                            #    lambda m: self._on_pose('robot2', m), #CHECK1
                                self._on_pose2,
                                10)

        # --- action clients ---
        self.ac = {
            'robot1': ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose'), #Used to send a goal to robotX
            'robot2': ActionClient(self, NavigateToPose, '/robot2/navigate_to_pose')
        }
        self.gh = {'robot1': None, 'robot2': None} # goal handle

        # --- state management ---
        self.state = 'INIT'
        self.evader_goal_sent = False
        self.round_auto_restart = True
        self.round_restart_delay = 5.0
        self.start_tol = 0.3 

        # --- timers ---
        self.create_timer(0.5, self._ensure_ready)
        self.create_timer(1.0, self._tick_chaser)  # Give new goal to chaser
        self.create_timer(0.3, self._tick_events)  # Check if event happened - tag/safe_zone
        
        # Manual start service #DEBUG
        self.create_service(Trigger, 'start_round', self._srv_start_round)
        
        self.get_logger().info("Orchestrator initialized")

    # def _on_pose(self, name, msg):
    #     self.pose[name] = msg.pose.position #CHECK1

    def _on_pose1(self, msg):
        self.pose['robot1'] = msg.pose.position

    def _on_pose2(self, msg):
        self.pose['robot2'] = msg.pose.position

    def _ensure_ready(self):
        if self.state != 'INIT':
            return
            
        # Wait to see if all servers are available (self.ac)
        servers_ready = all(
            self.ac[r].wait_for_server(timeout_sec=0.1) 
            for r in ['robot1', 'robot2']
        )
        
        # Wait for initial poses
        poses_ready = all(self.pose[r] is not None for r in ['robot1', 'robot2'])
        
        if servers_ready and poses_ready:
            self.get_logger().info("All systems ready - starting first round")
            self.start_round(swap_roles=False)  # First time don't swap

    def start_round(self, swap_roles=True):
        if self.state not in ('INIT', 'IDLE'):
            self.get_logger().debug(f'Ignoring start_round in state={self.state}')
            return
        if swap_roles:
            self.evader, self.chaser = self.chaser, self.evader
        self.get_logger().info(f'Round START → evader={self.evader}, chaser={self.chaser}')
        self.evader_goal_sent = False
        self.state = 'RUNNING'
        self._send_evader_safe()

    def _send_evader_safe(self):
        if self.evader_goal_sent or self.state != 'RUNNING':
            return
            
        x, y, yaw = self.safe[self.evader]
        self.get_logger().info(f'Sending evader {self.evader} to safe point ({x:.1f}, {y:.1f})')
        self._send_goal(self.evader, x, y, yaw)
        self.evader_goal_sent = True

    def _tick_chaser(self):
        if self.state != 'RUNNING':
            return
            
        if self.pose[self.evader] is None:
            self.get_logger().debug('No evader pose yet')
            return

        # Uvijek ažuriraj chaserov cilj ka evaderu
        evader_pos = self.pose[self.evader]
        self._cancel(self.chaser)
        self._send_goal(self.chaser, evader_pos.x, evader_pos.y, 0.0)

    def _tick_events(self):
        if self.state != 'RUNNING':
            return
            
        if self.pose[self.evader] is None or self.pose[self.chaser] is None:
            return

        # Provjeri tag
        distance = dist(self.pose[self.evader], self.pose[self.chaser])
        if distance <= self.tag_threshold:
            self.get_logger().info(f'TAG DETECTED! Distance: {distance:.2f}m')
            self._reset_round("tag")
            return

        # Provjeri da li je evader stigao u safe zonu
        evader_pos = self.pose[self.evader]
        safe_x, safe_y, _ = self.safe[self.evader]
        safe_distance = math.hypot(evader_pos.x - safe_x, evader_pos.y - safe_y)
        
        if safe_distance <= self.zone_r:
            self.get_logger().info(f'EVADER REACHED SAFE ZONE! Distance: {safe_distance:.2f}m')
            self._reset_round("safe_zone")

    def _reset_round(self, reason):
        self.get_logger().info(f'Resetting round: {reason}')
        self.state = 'RETURNING'
        
        # Zaustavi sve robote
        for robot in ['robot1', 'robot2']:
            self._cancel(robot)
        
        # Pošalji oba robota na startne pozicije
        for robot in ['robot1', 'robot2']:
            x, y, yaw = self.start[robot]
            self._send_goal(robot, x, y, yaw)
        
        # Pokreni timer za provjeru kada su oba robota na startu
        self.create_timer(0.5, self._check_both_at_start)

    def _check_both_at_start(self):
        if self.state != 'RETURNING':
            return
            
        both_at_start = True
        for robot in ['robot1', 'robot2']:
            if self.pose[robot] is None:
                both_at_start = False
                continue
                
            start_x, start_y, _ = self.start[robot]
            distance_to_start = math.hypot(
                self.pose[robot].x - start_x, 
                self.pose[robot].y - start_y
            )
            
            if distance_to_start > self.start_tol:
                both_at_start = False
                break
        
        if both_at_start:
            self.get_logger().info("Both robots at start positions")
            self.state = 'IDLE'
            
            if self.round_auto_restart:
                self.get_logger().info(f"Auto-restarting in {self.round_restart_delay}s")
                self.create_timer(
                    self.round_restart_delay, 
                    lambda: self.start_round(swap_roles=True)
                )
            else:
                self.get_logger().info("Waiting for manual start")

    def _send_goal(self, robot, x, y, yaw=0.0):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = q_yaw(yaw)
        
        future = self.ac[robot].send_goal_async(goal)
        future.add_done_callback(lambda f, r=robot: self._on_goal_response(r, f))

    def _on_goal_response(self, robot, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rejected for {robot}')
                return
                
            self.gh[robot] = goal_handle
            goal_handle.get_result_async().add_done_callback(
                lambda f, r=robot: self._on_goal_result(r, f)
            )
        except Exception as e:
            self.get_logger().error(f'Error in goal response for {robot}: {e}')

    def _on_goal_result(self, robot, future):
        try:
            result = future.result().result
            # Možete dodati logiku za obradu rezultata ako je potrebno
        except Exception as e:
            self.get_logger().error(f'Error in goal result for {robot}: {e}')

    def _cancel(self, robot):
        if self.gh[robot] is not None:
            try:
                self.gh[robot].cancel_goal_async()
                self.gh[robot] = None
            except Exception as e:
                self.get_logger().error(f'Error canceling goal for {robot}: {e}')

    def _srv_start_round(self, request, response):
        try:
            if self.state == 'IDLE':
                self.start_round(swap_roles=True)
                response.success = True
                response.message = "Round started successfully"
            else:
                response.success = False
                response.message = f"Cannot start round in current state: {self.state}"
        except Exception as e:
            response.success = False
            response.message = f"Error starting round: {e}"
        return response

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