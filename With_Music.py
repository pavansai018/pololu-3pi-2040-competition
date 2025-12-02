from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
import _thread

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()
buzzer = robot.Buzzer()

button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()

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

Kp_line = 3.0
Kd_line = 30.0
BASE_MAX_line = 2700

Kp_between = 2.0
Kd_between = 20.0
BASE_MAX_between = 2300

BETWEEN_TURN_SPEED = 1800
BETWEEN_TURN_TIME_MS = 150

KP_STEP = 0.1
KD_STEP = 0.5
TURN_SPEED_STEP = 100
TURN_TIME_STEP = 10

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

CORNER_TOTAL_THR = 3800
CORNER_COOLDOWN_MS = 600

between_turn_ms_remaining = 0
last_corner_ms = 0

music_enable = False

def music_task():
    global music_enable
    melody = "t170 l8 o5 g g g4 g g g4 a# a# a#4 a# a# a#4 g g g4 g g g4 f f f4 f f f4 g g g4 g g g4 d# d# d#4 d# d# d#4 g g g4 g g g4 a# a# a#4 a# a# a#4 g2"

    while True:
        if music_enable:
            buzzer.play_in_background(melody)
            while buzzer.is_playing() and music_enable:
                time.sleep_ms(10)
            if not music_enable:
                buzzer.off()
                time.sleep_ms(50)
        else:
            buzzer.off()
            time.sleep_ms(50)

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

def update_display(p, Kp_between, Kd_between, turn_spd, turn_tm, selected_param, mode):
    display.fill(0)
    if mode == MODE_WAIT:
        state = "WAIT 1st bump"
    elif mode == MODE_LINE:
        state = "MODE: LINE"
    else:
        state = "MODE: BETWEEN"
    display.text(state, 0, 0)
    display.text("A:Sel B:- C:+", 0, 10)

    if selected_param == 0:
        pb = ">"
        pdb = " "
        pts = " "
        ptt = " "
    elif selected_param == 1:
        pb = " "
        pdb = ">"
        pts = " "
        ptt = " "
    elif selected_param == 2:
        pb = " "
        pdb = " "
        pts = ">"
        ptt = " "
    else:
        pb = " "
        pdb = " "
        pts = " "
        ptt = ">"

    display.text("{}KpB:{:.2f}".format(pb, Kp_between), 0, 22)
    display.text("{}KdB:{:.2f}".format(pdb, Kd_between), 0, 32)
    display.text("{}TrnSpd:{}".format(pts, int(turn_spd)), 0, 42)
    display.text("{}TrnTm:{}".format(ptt, int(turn_tm)), 0, 52)
    display.show()

def main():
    global Kp_between, Kd_between, BETWEEN_TURN_SPEED, BETWEEN_TURN_TIME_MS
    global selected_param
    global last_a_ms, last_b_ms, last_c_ms
    global mode, bump_was_pressed
    global last_loop_ms, dt_nom_ms
    global between_turn_ms_remaining, last_corner_ms
    global music_enable

    calibrate()
    _thread.start_new_thread(music_task, ())
    time.sleep_ms(1)
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

        if any_pressed and not bump_was_pressed:
            if mode == MODE_WAIT:
                mode = MODE_LINE
                last_p = 0
                p = 0
                last_l = 2000
                between_turn_ms_remaining = 0
            elif mode == MODE_LINE:
                mode = MODE_BETWEEN
                last_p = 0
                p = 0
                between_turn_ms_remaining = 0
        bump_was_pressed = any_pressed

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
            p_raw = l - 2000
            Kp_use = Kp_line
            Kd_use = Kd_line
            BASE_MAX_use = BASE_MAX_line
        elif mode == MODE_BETWEEN:
            if between_turn_ms_remaining <= 0:
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
                if total > CORNER_TOTAL_THR and time.ticks_diff(now_ms, last_corner_ms) > CORNER_COOLDOWN_MS:
                    between_turn_ms_remaining = BETWEEN_TURN_TIME_MS
                    last_corner_ms = now_ms
            else:
                p_raw = 0
                between_turn_ms_remaining -= dt_ms
                if between_turn_ms_remaining < 0:
                    between_turn_ms_remaining = 0

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

        if mode == MODE_BETWEEN and between_turn_ms_remaining > 0:
            left_speed = -BETWEEN_TURN_SPEED
            right_speed = BETWEEN_TURN_SPEED
            if left_speed < -max_speed:
                left_speed = -max_speed
            if right_speed > max_speed:
                right_speed = max_speed

        if mode in (MODE_LINE, MODE_BETWEEN):
            motors.set_speeds(left_speed, right_speed)
            music_enable = True
        else:
            motors.off()
            music_enable = False

        if button_a.check():
            if time.ticks_diff(now_ms, last_a_ms) > DEBOUNCE_MS:
                selected_param = (selected_param + 1) % 4
                last_a_ms = now_ms

        if button_b.check():
            if time.ticks_diff(now_ms, last_b_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    Kp_between = max(0.0, Kp_between - KP_STEP)
                elif selected_param == 1:
                    Kd_between = max(0.0, Kd_between - KD_STEP)
                elif selected_param == 2:
                    BETWEEN_TURN_SPEED = max(0, BETWEEN_TURN_SPEED - TURN_SPEED_STEP)
                else:
                    BETWEEN_TURN_TIME_MS = max(10, BETWEEN_TURN_TIME_MS - TURN_TIME_STEP)
                last_b_ms = now_ms

        if button_c.check():
            if time.ticks_diff(now_ms, last_c_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    Kp_between += KP_STEP
                elif selected_param == 1:
                    Kd_between += KD_STEP
                elif selected_param == 2:
                    BETWEEN_TURN_SPEED = min(max_speed, BETWEEN_TURN_SPEED + TURN_SPEED_STEP)
                else:
                    BETWEEN_TURN_TIME_MS = BETWEEN_TURN_TIME_MS + TURN_TIME_STEP
                last_c_ms = now_ms

        if (now_ms % 100) < 10:
            update_display(p, Kp_between, Kd_between, BETWEEN_TURN_SPEED, BETWEEN_TURN_TIME_MS, selected_param, mode)

        time.sleep_ms(5)

main()
