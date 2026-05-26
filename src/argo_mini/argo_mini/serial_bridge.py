import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import time

WHEEL_RADIUS    = 0.0762
WHEEL_BASE      = 0.40
POLE_PAIRS      = 15
TICKS_PER_REV   = POLE_PAIRS * 6   # CHANGE mode: both edges × 3 phases = 90/rev
METERS_PER_TICK = (2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REV

DAC_STOP       = 0
DAC_LEFT_FWD   = 107
DAC_RIGHT_FWD  = 106
DAC_LEFT_TURN  = 105
DAC_RIGHT_TURN = 105


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('forward_only', False)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.forward_only = self.get_parameter('forward_only').value
        if self.forward_only:
            self.get_logger().info('forward_only=true: reverse commands blocked')

        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected to ESP32 on {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            raise

        self.odom_pub       = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub        = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_cb, 10)

        self.x          = 0.0
        self.y          = 0.0
        self.theta      = 0.0
        self.prev_left  = None
        self.prev_right = None
        self.last_time  = self.get_clock().now()

        self.create_timer(0.02, self.publish_tf)
        self.create_timer(0.01, self.read_serial)

        self.get_logger().info('SerialBridge ready.')

    def publish_tf(self):
        now = self.get_clock().now()
        qz  = math.sin(self.theta / 2.0)
        qw  = math.cos(self.theta / 2.0)
        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = 0.0
        tf.transform.rotation.y    = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)

    def cmd_cb(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z

        if self.forward_only and lin < 0.0:
            lin = 0.0

        if abs(lin) < 0.01 and abs(ang) < 0.01:
            # Stop
            dac_l, dac_r = DAC_STOP, DAC_STOP

        elif abs(lin) < 0.01:
            # Pure in-place rotation — proper tank turn using direction pins
            if ang > 0:  # counter-clockwise (left): left back, right forward
                dac_l, dac_r = -DAC_LEFT_TURN,  DAC_RIGHT_TURN
            else:        # clockwise (right): left forward, right back
                dac_l, dac_r =  DAC_LEFT_TURN, -DAC_RIGHT_TURN

        elif lin > 0:
            # Forward
            if abs(ang) < 0.01:
                dac_l, dac_r = DAC_LEFT_FWD, DAC_RIGHT_FWD
            elif ang > 0:   # forward-left: slow left, fast right
                dac_l, dac_r = DAC_LEFT_TURN, DAC_RIGHT_FWD
            else:            # forward-right: fast left, slow right
                dac_l, dac_r = DAC_LEFT_FWD, DAC_RIGHT_TURN

        else:
            # Reverse (lin < 0)
            if abs(ang) < 0.01:
                dac_l, dac_r = -DAC_LEFT_FWD, -DAC_RIGHT_FWD
            elif ang > 0:   # reverse-left: slow left, fast right (both negative)
                dac_l, dac_r = -DAC_LEFT_TURN, -DAC_RIGHT_FWD
            else:            # reverse-right: fast left, slow right (both negative)
                dac_l, dac_r = -DAC_LEFT_FWD, -DAC_RIGHT_TURN

        try:
            self.ser.write(f"V {dac_l} {dac_r}\n".encode())
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().warn(f'Write error: {e}')

    def read_serial(self):
        try:
            if self.ser.in_waiting > 200:
                self.ser.reset_input_buffer()
                return
            while self.ser.in_waiting:
                raw = self.ser.readline().decode(
                    'utf-8', errors='ignore').strip()
                if raw.startswith('O '):
                    parts = raw.split()
                    if len(parts) == 3:
                        self.update_odom(
                            int(parts[1]), int(parts[2]))
        except Exception as e:
            self.get_logger().warn(f'Read error: {e}')

    def update_odom(self, left_ticks, right_ticks):
        if self.prev_left is None:
            self.prev_left  = left_ticks
            self.prev_right = right_ticks
            return

        dl = (left_ticks  - self.prev_left)  * METERS_PER_TICK
        dr = (right_ticks - self.prev_right) * METERS_PER_TICK
        self.prev_left  = left_ticks
        self.prev_right = right_ticks

        d_center = (dl + dr) / 2.0
        d_theta  = (dr - dl) / WHEEL_BASE

        self.x     += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y     += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        v  = d_center / dt if dt > 0 else 0.0
        w  = d_theta  / dt if dt > 0 else 0.0
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        tf = TransformStamped()
        tf.header.stamp    = now.to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = 0.0
        tf.transform.rotation.y    = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp            = now.to_msg()
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = w
        odom.pose.covariance[0]      = 0.05
        odom.pose.covariance[7]      = 0.05
        odom.pose.covariance[35]     = 0.1
        odom.twist.covariance[0]     = 0.05
        odom.twist.covariance[35]    = 0.1
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b"S\n")
            node.ser.flush()
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
