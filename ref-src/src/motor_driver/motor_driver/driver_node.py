import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO
import time

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # GPIO Pin Definitions (update if needed)
        self.IN1 = 17
        self.IN2 = 18
        self.IN3 = 22
        self.IN4 = 23
        self.ENA = 27
        self.ENB = 24

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)

        # Set up PWM (for speed control)
        self.pwm_left = GPIO.PWM(self.ENA, 1000)
        self.pwm_right = GPIO.PWM(self.ENB, 1000)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        speed = min(100, max(0, abs(linear * 100)))
        turn = min(100, max(0, abs(angular * 100)))

        if linear > 0:
            self.move_forward(speed)
        elif linear < 0:
            self.move_backward(speed)
        elif angular > 0:
            self.turn_left(turn)
        elif angular < 0:
            self.turn_right(turn)
        else:
            self.stop()

    def move_forward(self, speed):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)

    def move_backward(self, speed):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)

    def turn_left(self, speed):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)

    def turn_right(self, speed):
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_left.ChangeDutyCycle(speed)
        self.pwm_right.ChangeDutyCycle(speed)

    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_left.ChangeDutyCycle(0)
        self.pwm_right.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
