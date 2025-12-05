from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import math  

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()

button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()

imu = robot.IMU()
imu.reset()
imu.enable_default()

buzzer = robot.Buzzer()

rgb_leds = robot.RGBLEDs()
rgb_leds.set_brightness(5)


# SONG = "t118 l8 o4 g a+ a+ a+ a+ g f d f g g a+ a+ a+ g f d f g a+ a+ a+ >c a+ g f d f g a+ a+ a+ g f d <b a g f d"
SONG = """
t220 l8
e g a4 a r a b o5 c4 c r c d o4 b4 b r a g a4 r e g a4 a r a b
o5 c4 c r c d o4 b4 b r a g a4 r e g a4 a r a o5 c d4 d r d e f4 f r
e d e o4 a4 r a b o5 c4 c r d4 e o4 a4 r a o5 c o4 b4 b r o5 c o4 a b4
r4 a4 a a b o5 c4 c r c d o4 b4 b r a g a4 r e g a4 a r a b o5 c4 c r
c d o4 b4 b r a g a4 r e g a4 a r a o5 c d4 d r d e f4 f r e d e o4 a4
r a b o5 c4 c r d4 e o4 a4 r a o5 c o4 b4 b r o5 c o4 a b4 r4 o5 e4
r r4 f4 r r4 e e r g r e d r r4 d4 r r4 c4 r r4 o4 b o5 c r o4 b r a2
o5 e4 r r4 f4 r r4 e e r g r e d r r4 d4 r r4 c4 r r4 o4 b o5 c r o4 b
r a2
"""


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


Kp_line = 2.0
Kd_line = 20.0
BASE_MAX_line = 2300

Kp_between = 2.0
Kd_between = 20.0
BASE_MAX_between = 2300

WHITE_THR_STEP = 50
TOTAL_THR_STEP = 50

selected_param = 0 

DEBOUNCE_MS = 200
last_a_ms = 0
last_b_ms = 0
last_c_ms = 0

LINE_TOTAL_MIN = 200  

P_FULL_ERROR = 1200
BASE_MIN_FRACTION = 0.2

MODE_WAIT = 0
MODE_LINE = 1
MODE_BETWEEN = 2

mode = MODE_WAIT
bump_was_pressed = False

last_loop_ms = 0
dt_nom_ms = 6.0

CORNER_WHITE_THR = 800      
CORNER_TOTAL_THR = 2750    
CORNER_COOLDOWN_MS = 600

between_turn_ms_remaining = 0
between_turn_dir = 0
last_corner_ms = 0

MIN_BETWEEN_TURN_INTERVAL_MS = 2000  
last_between_turn_ms = 0

gyro_kp = 140
gyro_kd = 4
gyro_max_speed = 1500
LINE_BLACK_SENSOR_THR = 700

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

turn_rate = 0.0       
robot_angle = 0.0     
last_time_gyro_reading = None

corner_count = 0           
line_corner_count = 0       
between_corner_count = 0    

music_on = False
SONG_START_MS = 0

# --- NEW: use accumulated dt for timing edge mode ---
LINE_EDGE_DELAY_MS = 9000      # 13 seconds in MODE_LINE
use_edge_mode = False           # False = centre follow, True = sensors 0 & 1
EDGE_TARGET_POS = 500           # 0..1000 target in edge mode
line_elapsed_ms = 0             # accumulated time spent in MODE_LINE (pre-edge)
# ---------------------------------------------------


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
    global use_edge_mode   # NEW

    display.fill(0)
    if mode == MODE_WAIT:
        state = "STOP (bump)"
    elif mode == MODE_LINE:
        if use_edge_mode:
            state = "MODE: LINEE"   # edge-follow mode
        else:
            state = "MODE: LINE"
    else:
        state = "MODE: BETWEEN"
    display.text(state, 0, 0)

    display.text("L:{} B:{}".format(line_corner_count, between_corner_count), 0, 10)

    display.text("A:Sel B:- C:+", 0, 20)

    if selected_param == 0:
        pw = ">"
        pb = " "
    else:
        pw = " "
        pb = ">"

    display.text("{}WThr:{}".format(pw, int(CORNER_WHITE_THR)), 0, 30)
    display.text("{}BThr:{}".format(pb, int(CORNER_TOTAL_THR)), 0, 40)

    display.show()


def update_leds(time_ms, mode):
    """
    Pirates of the Caribbean – LED sync
      - Uses song tempo t220, l8
      - MODE_LINE / MODE_BETWEEN: bright, more aggressive
      - MODE_WAIT: dimmer, softer
    """

    global SONG_START_MS
    t = time_ms - SONG_START_MS
    if t < 0:
        t = 0

    quarter_ms = 60000 / 220.0
    eighth_ms = quarter_ms / 2.0

    note_index = int(t // eighth_ms)
    beat_phase = (t % eighth_ms) / eighth_ms

    if mode in (MODE_LINE, MODE_BETWEEN):
        max_v = 255
        min_v = 80
    else:
        max_v = 120
        min_v = 40

    v = int(max_v - (max_v - min_v) * beat_phase * 0.9)

    hue_start = (note_index * 18) % 360
    hue_step = 50

    s = 230 + round(25 * math.cos(time_ms / 3000))

    for led in range(6):
        head = note_index % 6
        d = (led - head) % 6
        trail_factor = max(0.3, 1.0 - 0.25 * d)
        v_led = int(v * trail_factor)

        hue = hue_start + hue_step * led
        r, g, b = rgb_leds.hsv2rgb(hue, s, v_led)

        rgb_leds.set(led, [r, g // 3, b])

    rgb_leds.show()


def gyro_reset_angle():
    global robot_angle, last_time_gyro_reading
    robot_angle = 0.0
    last_time_gyro_reading = None


def gyro_update_angle():
    global robot_angle, last_time_gyro_reading, turn_rate

    if imu.gyro.data_ready():
        imu.gyro.read()
        turn_rate = imu.gyro.last_reading_dps[2]
        now = time.ticks_us()
        if last_time_gyro_reading is not None:
            dt = time.ticks_diff(now, last_time_gyro_reading)
            robot_angle += turn_rate * dt / 1_000_000.0
        last_time_gyro_reading = now


def gyro_turn_relative(target_angle_deg):
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
        time.sleep_ms(1)

    motors.off()


def handle_between_corner(now_ms):
    global corner_count, mode, between_corner_count
    global last_between_turn_ms

    if time.ticks_diff(now_ms, last_between_turn_ms) < MIN_BETWEEN_TURN_INTERVAL_MS:
        return

    last_between_turn_ms = now_ms

    corner_count += 1
    between_corner_count += 1
    print("Between corner #", corner_count)

    if corner_count < 20:
        gyro_turn_relative(90.0)
    else:
        motors.off()
        mode = MODE_WAIT
        corner_count = 0  


def main():
    global selected_param
    global last_a_ms, last_b_ms, last_c_ms
    global mode, bump_was_pressed
    global last_loop_ms, dt_nom_ms
    global between_turn_ms_remaining, between_turn_dir, last_corner_ms
    global corner_count, line_corner_count, between_corner_count
    global CORNER_WHITE_THR, CORNER_TOTAL_THR
    global music_on, SONG_START_MS
    global use_edge_mode, line_elapsed_ms   # NEW

    calibrate()
    time.sleep_ms(50)

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

        if any_pressed and not bump_was_pressed:
            if mode == MODE_WAIT:
                mode = MODE_LINE
                last_p = 0
                p = 0
                last_l = 2000
                between_turn_ms_remaining = 0
                between_turn_dir = 0
                corner_count = 0
                line_corner_count = 0
                between_corner_count = 0
                last_corner_ms = now_ms  

                # NEW: reset edge timer + flag
                use_edge_mode = False
                line_elapsed_ms = 0
                print("ENTER MODE_LINE: timer reset")

            elif mode == MODE_LINE:
                mode = MODE_BETWEEN
                last_p = 0
                p = 0
                between_turn_ms_remaining = 0
                between_turn_dir = 0
                corner_count = 0  
                between_corner_count = 0
                last_corner_ms = now_ms
                use_edge_mode = False   # next LINE run starts from centre
                print("ENTER MODE_BETWEEN from LINE")
                gyro_turn_relative(-90.0)

        bump_was_pressed = any_pressed

        if mode in (MODE_LINE, MODE_BETWEEN):
            if not music_on:
                buzzer.play_in_background(SONG)
                SONG_START_MS = time.ticks_ms() 
                music_on = True
            else:
                if not buzzer.is_playing():
                    buzzer.play_in_background(SONG)
        else:
            if music_on:
                buzzer.off()
                music_on = False

        update_leds(now_ms, mode)

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

        if mode == MODE_LINE:
            # NEW: accumulate dt while in MODE_LINE (before edge mode)
            if not use_edge_mode:
                line_elapsed_ms += dt_ms
                if line_elapsed_ms >= LINE_EDGE_DELAY_MS:
                    use_edge_mode = True
                    print("EDGE MODE ENABLED (", line_elapsed_ms, "ms): sensors 0 & 1")

            if use_edge_mode:
                # Use only first two sensors (0 & 1)
                s0 = line[0]
                s1 = line[1]
                total_01 = s0 + s1

                if total_01 > LINE_TOTAL_MIN:
                    pos01 = (0 * s0 + 1000 * s1) // total_01  # 0..1000
                    p_raw = pos01 - EDGE_TARGET_POS
                else:
                    # fallback if edge lost
                    p_raw = l - 2000
            else:
                # original centre-follow behaviour
                p_raw = l - 2000

            has_black = (
                line[0] > LINE_BLACK_SENSOR_THR or
                line[1] > LINE_BLACK_SENSOR_THR or
                line[2] > LINE_BLACK_SENSOR_THR or
                line[3] > LINE_BLACK_SENSOR_THR or
                line[4] > LINE_BLACK_SENSOR_THR
            )

            if (not has_black and
                    time.ticks_diff(now_ms, last_corner_ms) > CORNER_COOLDOWN_MS):
                motors.off()
                time.sleep_ms(5)
                line_corner_count += 1
                print("Line corner #", line_corner_count)
                gyro_turn_relative(-90.0)  
                last_corner_ms = now_ms
                last_p = 0
                p = 0
                last_l = 2000
                # NOTE: we do NOT reset use_edge_mode or line_elapsed_ms here.
                # Once edge mode is on, it stays for this LINE run.

            Kp_use = Kp_line
            Kd_use = Kd_line
            BASE_MAX_use = BASE_MAX_line

        elif mode == MODE_BETWEEN:
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

            if (total > CORNER_TOTAL_THR and
                    time.ticks_diff(now_ms, last_corner_ms) > CORNER_COOLDOWN_MS):
                motors.off()
                time.sleep_ms(5)
                motors.set_speeds(-700, -700)
                time.sleep_ms(300)
                motors.set_speeds(100, 100)
                time.sleep_ms(200)
                handle_between_corner(now_ms)
                last_corner_ms = now_ms
                last_p = 0
                p = 0
                last_l = 2000

            Kp_use = Kp_between
            Kd_use = Kd_between
            BASE_MAX_use = BASE_MAX_between

        else:
            p_raw = 0
            Kp_use = 0.0
            Kd_use = 0.0
            BASE_MAX_use = 0

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

        if mode in (MODE_LINE, MODE_BETWEEN):
            motors.set_speeds(left_speed, right_speed)
        else:
            motors.off()

        if button_a.check():
            if time.ticks_diff(now_ms, last_a_ms) > DEBOUNCE_MS:
                selected_param = (selected_param + 1) % 2
                last_a_ms = now_ms

        if button_b.check():
            if time.ticks_diff(now_ms, last_b_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    CORNER_WHITE_THR = max(0, CORNER_WHITE_THR - WHITE_THR_STEP)
                else:
                    CORNER_TOTAL_THR = max(0, CORNER_TOTAL_THR - TOTAL_THR_STEP)
                last_b_ms = now_ms

        if button_c.check():
            if time.ticks_diff(now_ms, last_c_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    CORNER_WHITE_THR = CORNER_WHITE_THR + WHITE_THR_STEP
                else:
                    CORNER_TOTAL_THR = CORNER_TOTAL_THR + TOTAL_THR_STEP
                last_c_ms = now_ms

        if (now_ms % 100) < 10:
            update_display(p, selected_param, mode)

        time.sleep_ms(1)


main()
