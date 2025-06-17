#!/usr/bin/env python3

import rospy
import robot_api
import sys
import select
import termios
import tty

msg = """
Control Your Fetch!
---------------------------
Moving around:
        w
   a    s    d

Space: force stop
i/k: increase/decrease only linear speed by 5 cm/s
u/j: increase/decrease only angular speed by 0.25 rads/s
anything else: stop smoothly

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (-1, 0),
}

speedBindings = {
    'i': (0.05, 0),
    'k': (-0.05, 0),
    'u': (0, 0.25),
    'j': (0, -0.25),
}

def getKey(timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f} m/s\tturn {turn:.2f} rad/s"

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('fetch_teleop_key')
    base = robot_api.Base()

    speed = 0.5  # m/s
    turn = 1.0   # rad/s
    x = 0
    th = 0
    status = 0
    count = 0
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings:
                x, th = moveBindings[key]
                count = 0
            elif key in speedBindings:
                ds, dt = speedBindings[key]
                speed += ds
                turn += dt
                count = 0
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ':
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
                base.stop()
            elif key == '\x03':  # CTRL+C
                break
            else:
                count += 1
                if count > 4:
                    x = 0
                    th = 0

            target_speed = speed * x
            target_turn = turn * th

            # Smooth transitions
            control_speed += max(min(target_speed - control_speed, 0.02), -0.02)
            control_turn += max(min(target_turn - control_turn, 0.1), -0.1)

            base.move(control_speed, control_turn)

    except Exception as e:
        rospy.logerr(f"Exception: {e}")
    finally:
        base.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
