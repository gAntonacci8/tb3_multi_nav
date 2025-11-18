#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        
        # Parameters
        self.declare_parameter('target_frame', 'robot1/base_link')
        self.declare_parameter('global_frame', 'map')
        
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.global_frame = self.get_parameter('global_frame').get_parameter_value().string_value
        
        # TF2 listener 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher 
        self.publisher_ = self.create_publisher(PoseStamped, '/robot1/target_pose', 10)
        
        # Timer 10 Hz
        self.timer = self.create_timer(0.1, self.publish_target_pose)
        self.get_logger().info('Target publisher is started. The frame is being tracked: ' + self.target_frame)

    def publish_target_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.target_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.05))

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.global_frame
            
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            
            self.publisher_.publish(pose_msg)
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Not possible to get transformation: {ex}', throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()