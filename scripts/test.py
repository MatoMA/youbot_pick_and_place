#!/usr/bin/env python
import roslib
roslib.load_manifest('youbot_pick_and_place')
import rospy
import sys, select, termios, tty, signal

class TimeoutException(Exception):
    pass

def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1) # this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
        key = sys.stdin.read(1)
       # print "Read key"
    except TimeoutException:
        # print "Timeout"
       return "-"
    finally:
       signal.signal(signal.SIGALRM, old_handler)
    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    while(1):
        settings = termios.tcgetattr(sys.stdin)
        key = getKey()
        if key == '1':
            print '1'
        elif key == '2':
            print '2'
        elif key == '3':
            print '3'
        elif key == 'q':
            break
