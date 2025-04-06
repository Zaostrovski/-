from time import sleep
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from serial import Serial
#import serial

class BaseSubscriber(Node):

    def __init__(self, serial: Serial):
        super().__init__('robot_base')
        self.subscription = self.create_subscription(
            Twist,
            'robot_cmd_vel',
            self.listener_callback,
            10)
        self.subscriptions
        self.ser = serial

    def listener_callback(self, twist: Twist):
        self.get_logger().info('"%s"' % twist)
        

        v = twist.linear.x
        phi = twist.angular.x

        # wheel: 1.6 об/сек
        # base : 
        # max ang speed: 3.06 rad/sed
        # max lin speed: 0.144 m/sec
        r = 90 / 1000
        b = 188 / 1000

        vl = (v - (phi * b) / 2) / r
        vr = (2 * v) / r - vl

        pwm1 = vl
        pwm2 = -vr
        # pwm1 = twist.linear.x
        # pwm2 = twist.linear.y

        msg1 = '{ \"op\": \"SetPwmB\", \"pwm\": ' + str(int(pwm1)) + ' }\n'
        msg1 = bytes(msg1, "ascii")
        print(msg1)
        self.ser.write(msg1) 
        sleep(0.02)
        
        msg2 = '{ \"op\": \"SetPwmA\", \"pwm\": ' + str(int(pwm2)) + ' }\n'
        msg2 = bytes(msg2, "ascii")
        print(msg2)
        self.ser.write(msg2) 
        sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    ser = Serial('/dev/ttyUSB1', 115200)  
    print(ser.name)         
    sub = BaseSubscriber(ser)
    rclpy.spin(sub)
    # close
    sub.destroy_node()
    ser.close() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()
