

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import _thread

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()

button_a = robot.ButtonA()

edition = editions.select()
if edition == "Standard":
    max_speed = 6000
    calibration_speed = 1000
    calibration_count = 100
elif edition == "Turtle":
    max_speed = 6000
    calibration_speed = 3000
    calibration_count = 100
elif edition == "Hyper":
    max_speed = 2000
    calibration_speed = 1000
    calibration_count = 100
    motors.flip_left(True)
    motors.flip_right(True)


BASE_SPEED = max_speed // 3      

KP = 1.2                       
KD = 4.0                         

# Max steering correction (in motor-speed units)
TURN_MAX = BASE_SPEED             


display.fill(0)
display.text("Line Follower", 0, 0)
display.text("Place on line", 0, 20)
display.text("and press A to", 0, 30)
display.text("calibrate.", 0, 40)
display.show()

while not button_a.check():
    pass

display.fill(0)
display.show()
time.sleep_ms(500)


motors.set_speeds(calibration_speed, -calibration_speed)
for i in range(calibration_count // 4):
    line_sensors.calibrate()

motors.off()
time.sleep_ms(200)

motors.set_speeds(-calibration_speed, calibration_speed)
for i in range(calibration_count // 2):
    line_sensors.calibrate()

motors.off()
time.sleep_ms(200)

motors.set_speeds(calibration_speed, -calibration_speed)
for i in range(calibration_count // 4):
    line_sensors.calibrate()

motors.off()


t1 = 0
t2 = time.ticks_us()
p = 0
line = []
starting = False
run_motors = False
last_update_ms = 0

def update_display():
    display.fill(0)
    display.text("Line Follower", 0, 0)
    if starting:
        display.text("Press A to stop", 0, 10)
    else:
        display.text("Press A to start", 0, 10)
    
    ms = (t2 - t1) / 1000
    display.text(f"Main loop: {ms:.1f}ms", 0, 20)
    display.text('p = ' + str(p), 0, 30)

    # 64-40 = 24
    scale = 24 / 1000

    print(line)
    if len(line) == 5:
        display.fill_rect(36, 64-int(line[0]*scale), 8, int(line[0]*scale), 1)
        display.fill_rect(48, 64-int(line[1]*scale), 8, int(line[1]*scale), 1)
        display.fill_rect(60, 64-int(line[2]*scale), 8, int(line[2]*scale), 1)
        display.fill_rect(72, 64-int(line[3]*scale), 8, int(line[3]*scale), 1)
        display.fill_rect(84, 64-int(line[4]*scale), 8, int(line[4]*scale), 1)

    display.show()

def follow_line():
    last_p = 0
    p_filt = 0
    global p, t1, t2, line, max_speed, run_motors
    while True:
        # Read sensors & measure loop time
        line = line_sensors.read_calibrated()[:]
        line_sensors.start_read()
        t1 = t2
        t2 = time.ticks_us()

        if line[1] < 700 and line[2] < 700 and line[3] < 700:
            if p < 0:
                l = 0
            else:
                l = 4000
        else:
            
            s = line[1] + line[2] + line[3] + line[4]
            if s == 0:
               
                l = 2000 + p
            else:
                l = (1000*line[1] + 2000*line[2] + 3000*line[3] + 4000*line[4]) // s

        # Proportional error: line position vs center (2000)
        p = l - 2000

        # Derivative (no dt; loop is fast & roughly constant)
        d = p - last_p
        last_p = p

        # Raw steering command (TURN)
        turn = KP * p + KD * d

        # Limit steering so we don't slam one wheel
        if turn > TURN_MAX:
            turn = TURN_MAX
        elif turn < -TURN_MAX:
            turn = -TURN_MAX

        # Compute left/right speeds around a base speed
        left  = BASE_SPEED + int(turn)
        right = BASE_SPEED - int(turn)

        # Clamp to motor limits [0, max_speed] (no reverse)
        if left < 0:       left = 0
        if left > max_speed:  left = max_speed
        if right < 0:      right = 0
        if right > max_speed: right = max_speed
 

        if run_motors:
            motors.set_speeds(left, right)
        else:
            motors.off()


# Sleep workaround for MicroPython thread bug
_thread.start_new_thread(follow_line, ())
time.sleep_ms(1)

while True:
    t = time.ticks_ms()

    # Update display ~10 Hz
    if time.ticks_diff(t, last_update_ms) > 100:
        last_update_ms = t
        update_display()

    # Button A to start/stop
    if button_a.check():
        if not starting:
            starting = True
            start_ms = t
        else:
            starting = False
            run_motors = False

    if starting and time.ticks_diff(t, start_ms) > 1000:
        run_motors = True
