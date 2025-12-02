from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()

button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()

# IMU for gyro-based turns
imu = robot.IMU()
imu.reset()
imu.enable_default()

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

# Line-following PD (black line)
Kp_line = 3.0
Kd_line = 30.0
BASE_MAX_line = 2700

# Between-lines PD (white corridor)
Kp_between = 2.0
Kd_between = 20.0
BASE_MAX_between = 2300

# Tuning increments for thresholds
WHITE_THR_STEP = 50
TOTAL_THR_STEP = 50

selected_param = 0  # 0 = CORNER_WHITE_THR, 1 = CORNER_TOTAL_THR

DEBOUNCE_MS = 200
last_a_ms = 0
last_b_ms = 0
last_c_ms = 0

LINE_TOTAL_MIN = 200  # used to decide if line visible

P_FULL_ERROR = 1200
BASE_MIN_FRACTION = 0.2

MODE_WAIT = 0
MODE_LINE = 1
MODE_BETWEEN = 2

mode = MODE_WAIT
bump_was_pressed = False

last_loop_ms = 0
dt_nom_ms = 6.0

# Corner detection:
# - MODE_LINE: "mostly white" -> low total
# - MODE_BETWEEN: "mostly black" -> high total
CORNER_WHITE_THR = 800      # tune via UI (MODE_LINE)
CORNER_TOTAL_THR = 3500     # you said you reduced this (MODE_BETWEEN)
CORNER_COOLDOWN_MS = 600

between_turn_ms_remaining = 0
between_turn_dir = 0
last_corner_ms = 0

# Extra protection: minimum interval between 90° turns in MODE_BETWEEN
MIN_BETWEEN_TURN_INTERVAL_MS = 2000  # 2 seconds
last_between_turn_ms = 0

# Gyro turning parameters (similar to Pololu example)
gyro_kp = 140
gyro_kd = 4
gyro_max_speed = 1500

# Adjust gyro params depending on edition (optional tuning)
if edition == "Standard":
    gyro_max_speed = gyro_max_speed
    gyro_kp = 140
    gyro_kd = 4
elif edition == "Turtle":
    gyro_max_speed = 6000
    gyro_kp = 350
    gyro_kd = 7
elif edition == "Hyper":
    gyro_max_speed = 1500
    gyro_kp = 140
    gyro_kd = 4

# Gyro state
turn_rate = 0.0       # degrees per second
robot_angle = 0.0     # integrated angle
last_time_gyro_reading = None

# Corner counting
corner_count = 0            # used to stop after 6 in MODE_BETWEEN
line_corner_count = 0       # how many corners MODE_LINE has detected
between_corner_count = 0    # how many corners MODE_BETWEEN has detected


def calibrate():
    display.fill(0)
    display.text("Cal Line", 0, 0)
    display.show()

    motors.set_speeds(calibration_speed, -calibration_speed)
    for _ in range(calibration_count // 4):
        line_sensors.calibrate()
        time.sleep_ms(10)
    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(-calibration_speed, calibration_speed)
    for _ in range(calibration_count // 2):
        line_sensors.calibrate()
        time.sleep_ms(10)
    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(calibration_speed, -calibration_speed)
    for _ in range(calibration_count // 4):
        line_sensors.calibrate()
        time.sleep_ms(10)
    motors.off()
    time.sleep_ms(200)

    display.fill(0)
    display.text("Cal Bump", 0, 0)
    display.show()
    bump_sensors.calibrate()
    time.sleep_ms(300)

    display.fill(0)
    display.text("A:Sel B:- C:+", 0, 0)
    display.text("1:LINE 2:BETW", 0, 10)
    display.show()
    time.sleep_ms(500)


def update_display(p, selected_param, mode):
    """
    UI shows and edits corner thresholds:
      - CORNER_WHITE_THR (MODE_LINE)
      - CORNER_TOTAL_THR (MODE_BETWEEN)
    """
    global line_corner_count, between_corner_count
    global CORNER_WHITE_THR, CORNER_TOTAL_THR

    display.fill(0)
    if mode == MODE_WAIT:
        state = "STOP (bump)"
    elif mode == MODE_LINE:
        state = "MODE: LINE"
    else:
        state = "MODE: BETWEEN"
    display.text(state, 0, 0)

    # Show corner counts for both modes
    display.text("L:{} B:{}".format(line_corner_count, between_corner_count), 0, 10)

    display.text("A:Sel B:- C:+", 0, 20)

    # Selection marker for which threshold we’re editing
    if selected_param == 0:
        pw = ">"
        pb = " "
    else:
        pw = " "
        pb = ">"

    display.text("{}WThr:{}".format(pw, int(CORNER_WHITE_THR)), 0, 30)
    display.text("{}BThr:{}".format(pb, int(CORNER_TOTAL_THR)), 0, 40)

    display.show()


# ---------- Gyro helpers ----------

def gyro_reset_angle():
    global robot_angle, last_time_gyro_reading
    robot_angle = 0.0
    last_time_gyro_reading = None


def gyro_update_angle():
    """
    Read gyro Z axis and integrate to update robot_angle in degrees.
    Called repeatedly during a turn.
    """
    global robot_angle, last_time_gyro_reading, turn_rate

    if imu.gyro.data_ready():
        imu.gyro.read()
        turn_rate = imu.gyro.last_reading_dps[2]  # yaw in deg/s
        now = time.ticks_us()
        if last_time_gyro_reading is not None:
            dt = time.ticks_diff(now, last_time_gyro_reading)
            robot_angle += turn_rate * dt / 1_000_000.0
        last_time_gyro_reading = now


def gyro_turn_relative(target_angle_deg):
    """
    Blocking relative turn using gyro.
    Positive = left turn (+90), negative = right turn (-90).
    """
    global robot_angle

    motors.off()
    gyro_reset_angle()
    last_time_far_from_target = time.ticks_ms()

    while True:
        gyro_update_angle()
        error = target_angle_deg - robot_angle

        far_from_target = abs(error) > 3.0
        if far_from_target:
            last_time_far_from_target = time.ticks_ms()
        else:
            if time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 250:
                break

        turn_speed = error * gyro_kp - turn_rate * gyro_kd

        if turn_speed > gyro_max_speed:
            turn_speed = gyro_max_speed
        if turn_speed < -gyro_max_speed:
            turn_speed = -gyro_max_speed

        motors.set_speeds(-int(turn_speed), int(turn_speed))
        time.sleep_ms(5)

    motors.off()


def handle_between_corner(now_ms):
    """
    Called whenever a corner is detected in MODE_BETWEEN.

    Pattern:
      - Second bump already did a -90° (right) when entering MODE_BETWEEN.
      - Here, in MODE_BETWEEN:
          Corner 1–5: +90° (left) turns
          Corner 7:   stop robot

    Extra guard:
      - Do NOT allow more than one 90° turn within MIN_BETWEEN_TURN_INTERVAL_MS.
    """
    global corner_count, mode, between_corner_count
    global last_between_turn_ms

    # Rate limit: skip if last turn was less than 2 seconds ago
    if time.ticks_diff(now_ms, last_between_turn_ms) < MIN_BETWEEN_TURN_INTERVAL_MS:
        return

    last_between_turn_ms = now_ms

    corner_count += 1
    between_corner_count += 1
    print("Between corner #", corner_count)

    if corner_count < 7:
        # Corners 1–5: +90° left turns
        gyro_turn_relative(90.0)
    else:
        # 6th corner: stop robot (go back to WAIT, ready for new run)
        motors.off()
        mode = MODE_WAIT
        corner_count = 0  # reset logical cycle count


# ---------- Main loop ----------

def main():
    global selected_param
    global last_a_ms, last_b_ms, last_c_ms
    global mode, bump_was_pressed
    global last_loop_ms, dt_nom_ms
    global between_turn_ms_remaining, between_turn_dir, last_corner_ms
    global corner_count, line_corner_count, between_corner_count
    global CORNER_WHITE_THR, CORNER_TOTAL_THR

    calibrate()
    time.sleep_ms(500)

    last_p = 0
    p = 0
    last_l = 2000

    last_loop_ms = time.ticks_ms()
    line_sensors.start_read()

    while True:
        now_ms = time.ticks_ms()
        dt_ms = time.ticks_diff(now_ms, last_loop_ms)
        if dt_ms <= 0:
            dt_ms = 1
        last_loop_ms = now_ms

        dt_nom_ms = 0.99 * dt_nom_ms + 0.01 * dt_ms

        bump_sensors.read()
        left_pressed = bump_sensors.left_is_pressed()
        right_pressed = bump_sensors.right_is_pressed()
        any_pressed = left_pressed or right_pressed

        # Handle bump presses for mode switching
        if any_pressed and not bump_was_pressed:
            if mode == MODE_WAIT:
                # First bump -> go to LINE mode
                mode = MODE_LINE
                last_p = 0
                p = 0
                last_l = 2000
                between_turn_ms_remaining = 0
                between_turn_dir = 0
                corner_count = 0
                line_corner_count = 0
                between_corner_count = 0
                last_corner_ms = now_ms  # reset cooldown
            elif mode == MODE_LINE:
                # Second bump -> go to BETWEEN mode and take -90° right turn
                mode = MODE_BETWEEN
                last_p = 0
                p = 0
                between_turn_ms_remaining = 0
                between_turn_dir = 0
                corner_count = 0  # start counting between-corners from here
                between_corner_count = 0
                last_corner_ms = now_ms
                gyro_turn_relative(-90.0)

        bump_was_pressed = any_pressed

        # Read line sensors
        line = line_sensors.read_calibrated()
        line_sensors.start_read()

        total = line[0] + line[1] + line[2] + line[3] + line[4]

        if total > LINE_TOTAL_MIN:
            weighted_sum = (
                0 * line[0]
                + 1000 * line[1]
                + 2000 * line[2]
                + 3000 * line[3]
                + 4000 * line[4]
            )
            l = weighted_sum // total
            last_l = l
        else:
            l = last_l

        # Mode-specific error computation + corners
        if mode == MODE_LINE:
            # MODE_LINE: robot chases black line
            p_raw = l - 2000

            # Corner condition in MODE_LINE: "whenever it sees most white"
            if (total < CORNER_WHITE_THR and
                    time.ticks_diff(now_ms, last_corner_ms) > CORNER_COOLDOWN_MS):
                motors.off()
                time.sleep_ms(50)
                line_corner_count += 1
                print("Line corner #", line_corner_count)
                gyro_turn_relative(-90.0)  # all right turns in MODE_LINE
                last_corner_ms = now_ms
                # After turn, reset controller memory
                last_p = 0
                p = 0
                last_l = 2000

            Kp_use = Kp_line
            Kd_use = Kd_line
            BASE_MAX_use = BASE_MAX_line

        elif mode == MODE_BETWEEN:
            # MODE_BETWEEN: robot chases white corridor
            # Invert readings for "white center" tracking between two lines
            w0 = 1000 - line[0]
            w1 = 1000 - line[1]
            w2 = 1000 - line[2]
            w3 = 1000 - line[3]
            w4 = 1000 - line[4]
            total_w = w0 + w1 + w2 + w3 + w4
            if total_w < 1:
                total_w = 1
            pos = (0 * w0 + 1000 * w1 + 2000 * w2 + 3000 * w3 + 4000 * w4) // total_w
            p_raw = pos - 2000

            # Corner condition in MODE_BETWEEN: "most black values"
            if (total > CORNER_TOTAL_THR and
                    time.ticks_diff(now_ms, last_corner_ms) > CORNER_COOLDOWN_MS):
                motors.off()
                time.sleep_ms(50)
                handle_between_corner(now_ms)
                last_corner_ms = now_ms
                # After turn, reset controller memory
                last_p = 0
                p = 0
                last_l = 2000

            Kp_use = Kp_between
            Kd_use = Kd_between
            BASE_MAX_use = BASE_MAX_between

        else:
            # MODE_WAIT: no movement
            p_raw = 0
            Kp_use = 0.0
            Kd_use = 0.0
            BASE_MAX_use = 0

        # PD controller
        if mode in (MODE_LINE, MODE_BETWEEN):
            p = p_raw
            d_raw = p - last_p
            last_p = p
            d_scaled = d_raw * (dt_nom_ms / dt_ms)
        else:
            p = 0
            last_p = 0
            d_scaled = 0

        base_min = int(BASE_MAX_use * BASE_MIN_FRACTION)
        if base_min < 0:
            base_min = 0
        if base_min > BASE_MAX_use:
            base_min = BASE_MAX_use

        abs_p = abs(p)
        if abs_p >= P_FULL_ERROR:
            base = base_min
        else:
            base = BASE_MAX_use - (BASE_MAX_use - base_min) * abs_p / P_FULL_ERROR
            base = int(base)

        turn = Kp_use * p + Kd_use * d_scaled

        if turn > base:
            turn = base
        elif turn < -base:
            turn = -base

        left_speed = base + int(turn)
        right_speed = base - int(turn)

        if left_speed < 0:
            left_speed = 0
        if left_speed > max_speed:
            left_speed = max_speed
        if right_speed < 0:
            right_speed = 0
        if right_speed > max_speed:
            right_speed = max_speed

        # Drive motors (gyro turns themselves set speeds inside their loop)
        if mode in (MODE_LINE, MODE_BETWEEN):
            motors.set_speeds(left_speed, right_speed)
        else:
            motors.off()

        # Button parameter tuning
        if button_a.check():
            if time.ticks_diff(now_ms, last_a_ms) > DEBOUNCE_MS:
                # Only two params now: 0 = white threshold, 1 = black threshold
                selected_param = (selected_param + 1) % 2
                last_a_ms = now_ms

        if button_b.check():
            if time.ticks_diff(now_ms, last_b_ms) > DEBOUNCE_MS:
                # Decrease selected threshold
                if selected_param == 0:
                    CORNER_WHITE_THR = max(0, CORNER_WHITE_THR - WHITE_THR_STEP)
                else:
                    CORNER_TOTAL_THR = max(0, CORNER_TOTAL_THR - TOTAL_THR_STEP)
                last_b_ms = now_ms

        if button_c.check():
            if time.ticks_diff(now_ms, last_c_ms) > DEBOUNCE_MS:
                # Increase selected threshold
                if selected_param == 0:
                    CORNER_WHITE_THR = CORNER_WHITE_THR + WHITE_THR_STEP
                else:
                    CORNER_TOTAL_THR = CORNER_TOTAL_THR + TOTAL_THR_STEP
                last_c_ms = now_ms

        # Update display roughly every 100 ms
        if (now_ms % 100) < 10:
            update_display(p, selected_param, mode)

        time.sleep_ms(5)


main()
