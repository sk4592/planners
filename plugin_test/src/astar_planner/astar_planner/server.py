import rclpy
from rclpy.node import Node
from astar_srv.srv import Astar

class Server(Node):
    def __init__(self):
        super().__init__('Astar_planner')
        self.create_service(Astar, 'astar_planner', self.callback)
        self.get_logger().info('initializing the server')

    def callback(self, req, res):
        self.get_logger().info('Running Astar planner')

        self.get_logger().info(f"received data {req}")
        
        return res

def main():
    rclpy.init()

    server = Server()

    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == "__main__":
    main()