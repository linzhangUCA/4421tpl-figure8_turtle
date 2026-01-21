import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from turtlesim.srv import SetPen, TeleportAbsolute
from math import pi


class Figure8Node(Node):
    def __init__(self):
        super().__init__("fig8_node")
        # Draw boundary
        self.set_red_cli = self.create_client(SetPen, "/turtle1/set_pen")
        while not self.set_red_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("set pen service not available, waiting again...")
        self.set_red_request(255, 2)

        self.teleport_cli = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        while not self.teleport_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("teleport service not available, waiting again...")
        self.tp_req = TeleportAbsolute.Request()
        bounds = (
            (6.544545, 5.544445),
            (6.544545, 7.544545),
            (4.544345, 7.544545),
            (4.544345, 5.544445),
            (7.544545, 5.544445),
            (7.544545, 1.544345),
            (3.544345, 1.544345),
            (3.544345, 5.544445),
            (5.544445, 5.544445),
        )
        for b in bounds:
            self.teleport_request(b[0], b[1])

        self.set_red_request(0, 5)

        ### START CODING HERE ###
        self.pose_listener = None  # set up subscriber
        self.cmd_talker = None  # set up publisher
        self.cmd_pub_timer = None  # set up timer
        # Constants
        self.ang_vel = None  # angular velocity
        # Variables
        self.circle_counter = None  # pair with pub freq to switch ang_vel dir
        self.is_clockwise = None  # flag for ang_vel direction

        ### END CODING HERE ###

    ### START CODING HERE ###
    def subscriber_callback_function(
        self, pose_msg
    ):  # feel free to rename the function
        self.get_logger().info(
            f"Turtle's pose: \nposition x: {None}, position y: {None}, orientation z: {None}"
        )  # hint: extract pos.x, pos.y, ori.z from the "pose_msg"

    ### END CODING HERE ###

    ### START CODING HERE ###
    def timer_callback_function(self):  # feel free to rename the function
        twist_msg = None
        if self.circle_counter % None == 0:  # change ang_vel dir after a lap
            self.is_clockwise = not self.is_clockwise

        if self.is_clockwise == True:  # update twist message
            twist_msg.linear.x = None
            twist_msg.angular.z = None
        else:
            twist_msg.linear.x = None
            twist_msg.angular.z = None
        self.cmd_talker.publish(None)
        self.circle_counter = None  # don't forget to increment counter

        self.get_logger().debug(f"Velocity command: {twist_msg}")  # "debug" <-> "info"

    ### END CODING HERE ###

    # Service clients for drawing boundaries
    def set_red_request(self, r, width):
        "Set pen color to red, width to 2"
        req = SetPen.Request()
        req.r = r
        req.width = width
        # Call the service
        future = self.set_red_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # Check if the service call was successful
        if future.result() is not None:
            self.get_logger().info(f"Set pen color to {req}")
        else:
            self.get_logger().error("Failed to call service /turtle1/set_pen")

    def teleport_request(self, dest_x, dest_y):
        "Teleport turtle to dest_x, dest_y"
        self.tp_req = TeleportAbsolute.Request()
        self.tp_req.x = dest_x
        self.tp_req.y = dest_y
        # Call the service
        future = self.teleport_cli.call_async(self.tp_req)
        rclpy.spin_until_future_complete(self, future)
        # Check if the service call was successful
        if future.result() is not None:
            self.get_logger().info(f"Teleport to {self.tp_req}")
        else:
            self.get_logger().error("Failed to call service /turtle1/teleport_absolute")


def main(args=None):
    rclpy.init(args=args)
    turtle8 = Figure8Node()
    rclpy.spin(turtle8)
    turtle8.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
