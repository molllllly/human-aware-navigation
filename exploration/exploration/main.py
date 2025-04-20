import rclpy
from .explorer import Explorer

def main():
    rclpy.init()
    node = Explorer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()