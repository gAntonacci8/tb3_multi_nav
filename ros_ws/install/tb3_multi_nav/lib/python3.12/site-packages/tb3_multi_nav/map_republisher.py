import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # tipo della mappa
from rclpy.qos import QoSProfile, DurabilityPolicy #to get TRANSIENT_LOCAL as qos compatible 


class MapBroadcaster(Node):
    def __init__(self):
        super().__init__('map_broadcaster')
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.pub1 = self.create_publisher(OccupancyGrid, '/robot1/map', qos) #was 10, not qos
        self.pub2 = self.create_publisher(OccupancyGrid, '/robot2/map', qos) #was 10, not qos

    def map_callback(self, msg):
        # ripubblica senza modifiche
        self.pub1.publish(msg)
        self.pub2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
