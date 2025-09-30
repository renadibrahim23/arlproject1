import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleDrawer(Node):
    def __init__(self):
        super().__init__('turtle_drawer')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def move(self, linear=0.0, angular=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        time.sleep(duration)
        # stop
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def draw_square(self):
        for _ in range(4):
            self.move(linear=2.0, duration=2.0)
            self.move(angular=1.57, duration=1.0)  # ~90°

    def draw_triangle(self):
        for _ in range(3):
            self.move(linear=2.0, duration=2.0)
            self.move(angular=2.09, duration=1.0)  # ~120°

    def draw_circle(self):
        self.move(linear=2.0, angular=1.0, duration=6.0)

def main():
    rclpy.init()
    turtle = TurtleDrawer()

    choice = input("Choose a shape (square, triangle, circle): ").strip().lower()
    if choice == "square":
        turtle.draw_square()
    elif choice == "triangle":
        turtle.draw_triangle()
    elif choice == "circle":
        turtle.draw_circle()
    else:
        print("Invalid choice!")

    turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
