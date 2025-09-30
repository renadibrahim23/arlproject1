import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen
import time
import math


class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /clear service...")
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /teleport_absolute service...")
        while not self.pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_pen service...")

    def set_pen(self, r=0, g=0, b=255, width=2, off=False):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = bool(off)
        self.pen_client.call_async(req)

    def reset_turtle(self):
        self.clear_client.call_async(Empty.Request())
        self.set_pen(off=True)
        teleport_req = TeleportAbsolute.Request()
        teleport_req.x = 5.5
        teleport_req.y = 5.5
        teleport_req.theta = 0.0  # facing right (east) for polygons
        self.teleport_client.call_async(teleport_req)
        time.sleep(0.5)
        self.set_pen(off=False)
        print("✅ Turtle reset to center, facing right, screen cleared.")

    def move(self, linear, angular, duration):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)
        time.sleep(duration)
        self.publisher.publish(Twist())

    def draw_star(self):
        turn_angle = 2 * math.pi * 2 / 5
        for _ in range(5):
            self.move(2.0, 0.0, 1.0)
            self.move(0.0, turn_angle, 1.0)

    def draw_spiral(self):
        speed = 1.0
        for _ in range(30):
            self.move(speed, 1.0, 0.5)
            speed += 0.05

    def make_shape(self, numSides):
        angle = (2 * math.pi) / numSides
        for _ in range(numSides):
            self.move(2.0, 0.0, 1.0)
            self.move(0.0, angle, 1.0)

    def draw_nested_polygons(self):
        for sides in range(3, 9):
            #self.reset_turtle()  # reset before each polygon
            self.make_shape(sides)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDrawer()

    while True:
        shape = input("\nEnter shape (star / spiral / nested / quit): ").strip().lower()
        if shape == "quit":
            print("Exiting...")
            break

        node.reset_turtle()

        if shape == "star":
            node.draw_star()
        elif shape == "spiral":
            node.draw_spiral()
        elif shape == "nested":
            node.draw_nested_polygons()
        else:
            print("Unknown shape:", shape)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
