#!/usr/bin/env python3

################################################################################
# AutoDRIVE - RoboRacer Teleoperation Panel with Feedback
################################################################################

# ROS 2 module imports
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32, Bool

# Python module imports
import os
import select
import sys
if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Parameters
DRIVE_LIMIT = 1.0
STEER_LIMIT = 1.0
DRIVE_STEP_SIZE = 0.2
STEER_STEP_SIZE = 0.2

# Global variable to store feedback from the subscriber
current_speed = 0.0

# Information
info = """
-----------------------------------------
AutoDRIVE - RoboRacer Teleoperation Panel
-----------------------------------------

               Q   W   E
               A   S   D
                   X
                   R

W/S : Increase/decrease drive command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and zero steer
R   : Soft-reset the simulator
Press CTRL+C to quit

NOTE: Press keys within this terminal
-----------------------------------------
"""

break_flag = False

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low_bound, high_bound):
    if input < low_bound: return low_bound
    if input > high_bound: return high_bound
    return input

def bound_steer(steer_cmd):
    return constrain(steer_cmd, -STEER_LIMIT, STEER_LIMIT)

def bound_drive(drive_cmd):
    return constrain(drive_cmd, -DRIVE_LIMIT, DRIVE_LIMIT)

# Callback for the subscription
def break_callback(msg):
    # key = 'e'
    global break_flag
    break_flag = msg.data

################################################################################


def main():
    global break_flag
    global current_speed
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS 2 infrastructure
    rclpy.init()
    qos = QoSProfile(depth=1)
    node = rclpy.create_node('teleop_keyboard')
    
    # Publishers
    pub_steering_command = node.create_publisher(Float32, '/autodrive/roboracer_1/steering_command', qos)
    pub_throttle_command = node.create_publisher(Float32, '/autodrive/roboracer_1/throttle_command_raw', qos)
    pub_reset_command = node.create_publisher(Bool, '/autodrive/reset_command', qos)

    # Subscription
    # NOTE: Ensure the topic name matches your simulator setup
    e_break = node.create_subscription(Bool, 'autodrive/roboracer_1/emergency_break', break_callback, qos)

    # Initialize messages and local variables
    throttle_msg = Float32()
    steering_msg = Float32()
    reset_msg = Bool()
    
    throttle = 0.0
    steering = 0.0
    reset_flag = False

    try:
        print(info)

        while rclpy.ok():
            # 1. Spin the node once to process any incoming speed messages
            # This allows speed_callback to update the current_speed variable
            rclpy.spin_once(node, timeout_sec=0.01)

            # 2. Get keyboard input
            key = get_key(settings)
            changed = False
            if ( break_flag == True):
                key = 'e'
                break_flag = False

            if key == 'w' :
                throttle = bound_drive(throttle + DRIVE_STEP_SIZE)
                changed = True
            elif key == 's' :
                throttle = bound_drive(throttle - DRIVE_STEP_SIZE)
                changed = True
            elif key == 'a' :
                steering = bound_steer(steering + STEER_STEP_SIZE)
                changed = True
            elif key == 'd' :
                steering = bound_steer(steering - STEER_STEP_SIZE)
                changed = True
            elif key == 'q' :
                steering = 0.0
                changed = True
            elif key == 'e' :
                throttle = 0.0
                changed = True
            elif key == 'x' :
                throttle = 0.0
                steering = 0.0
                changed = True
            elif key == 'r' :
                throttle = 0.0
                steering = 0.0
                changed = True
                reset_flag = True
            elif key == '\x03': # CTRL+C
                break
            
            # 3. Print status (Optional: show current speed in terminal)
            # Use \r to overwrite the line so it doesn't scroll forever
            sys.stdout.write(f"\rThrottle: {throttle:.2f} | Steer: {steering:.2f} | Speed: {current_speed:.2f}    ")
            sys.stdout.flush()

            # 4. Generate and publish control messages

            throttle_msg.data = float(throttle)
            steering_msg.data = float(steering)
            reset_msg.data = reset_flag

            pub_throttle_command.publish(throttle_msg)
            pub_steering_command.publish(steering_msg)
            pub_reset_command.publish(reset_msg)
                
            reset_flag = False

    except Exception as e:
        print(f"\n{e}")

    finally:
        # Stop the car on exit
        throttle_msg.data = 0.0
        steering_msg.data = 0.0
        reset_msg.data = False
        pub_throttle_command.publish(throttle_msg)
        pub_steering_command.publish(steering_msg)
        pub_reset_command.publish(reset_msg)
        
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()