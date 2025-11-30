from pololu_3pi_2040_robot import robot
import time
import sys
import uselect

motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()

max_speed = 6000

left_cmd = 0
right_cmd = 0

poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v

def handle_command(line):
    global left_cmd, right_cmd
    line = line.strip()
    if not line:
        return
    parts = line.split()
    if parts[0].upper() == "M" and len(parts) >= 3:
        try:
            l = int(parts[1])
            r = int(parts[2])
            l = clamp(l, -max_speed, max_speed)
            r = clamp(r, -max_speed, max_speed)
            left_cmd = l
            right_cmd = r
        except:
            return
    elif parts[0].upper() == "STOP":
        left_cmd = 0
        right_cmd = 0

bump_sensors.calibrate()

last_log_ms = 0

while True:
    now_ms = time.ticks_ms()

    for obj, ev in poll.poll(0):
        if obj is sys.stdin and ev & uselect.POLLIN:
            try:
                cmd_line = sys.stdin.readline()
            except:
                cmd_line = ""
            if cmd_line:
                handle_command(cmd_line)

    motors.set_speeds(left_cmd, right_cmd)

    line_raw = line_sensors.read()
    bump_sensors.read()
    bumpL = 1 if bump_sensors.left_is_pressed() else 0
    bumpR = 1 if bump_sensors.right_is_pressed() else 0

    if time.ticks_diff(now_ms, last_log_ms) >= 20:
        last_log_ms = now_ms
        l0 = int(line_raw[0])
        l1 = int(line_raw[1])
        l2 = int(line_raw[2])
        l3 = int(line_raw[3])
        l4 = int(line_raw[4])
        print(
            "LOG",
            now_ms,
            left_cmd,
            right_cmd,
            l0,
            l1,
            l2,
            l3,
            l4,
            bumpL,
            bumpR,
        )
