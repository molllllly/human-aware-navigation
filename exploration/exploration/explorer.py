import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from project_interface.srv import ExploreGoal

import numpy as np

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer started!")
        self.srv = self.create_service(ExploreGoal, 'explorer/get_next_goal', self.handle_request)

    async def handle_request(self, request, response: Pose):
        self.get_logger().info("Waiting for /map...")
        map_msg = wait_for_message(self, OccupancyGrid, '/map')

        result : tuple[bool, OccupancyGrid] = wait_for_message(OccupancyGrid, self.ros_node, "/odom", time_to_wait=10)
        success, map_msg = result

        if not success:
            self.get_logger().error("Could not get current map. ERROR!")
            response.position.x = 0
            response.position.y = 0
            return

        self.get_logger().info("Map received.")

        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin

        map_array = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # TODO: Replace this with actual frontier/exploration logic
        # Example: just pick a dummy unexplored point
        goal = Pose()
        goal.position.x = origin.position.x + width * resolution / 2
        goal.position.y = origin.position.y + height * resolution / 2
        goal.orientation.w = 1.0

        response.next_goal = goal
        return response


