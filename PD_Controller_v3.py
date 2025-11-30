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

Kp_between = 1.5
Kd_between = 20.0
BASE_MAX_between = 2300

P_FULL_ERROR_LINE = 1200
P_FULL_ERROR_BETWEEN = 600
BETWEEN_DEADBAND = 100
BASE_MIN_FRACTION = 0.2

KP_STEP = 0.1
KD_STEP = 1.0
BASE_STEP = 100
P_FULL_STEP = 50
DEADBAND_STEP = 10
CORNER_THR_STEP = 20
TURN_MS_STEP = 10

DEBOUNCE_MS = 200
last_a_ms = 0
last_b_ms = 0
last_c_ms = 0

LINE_TOTAL_MIN = 200

MODE_WAIT = 0
MODE_LINE = 1
MODE_BETWEEN = 2

mode = MODE_WAIT
bump_was_pressed = False

last_loop_ms = 0
dt_nom_ms = 6.0

HISTORY_LEN = 200
history = [[0, 0, 0, 0, 0] for _ in range(HISTORY_LEN)]
hist_idx = 0
hist_count = 0

CORNER_BLACK_THR = 800
CORNER_DEBOUNCE = 3
corner_high_count = 0
CORNER_LOOKBACK = 80
TURN_CORNER_MS = 280

PARAM_COUNT = 11
selected_param_idx = 0


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
    display.text("A:Next B:- C:+", 0, 0)
    display.text("1:LINE 2:BETW", 0, 10)
    display.show()
    time.sleep_ms(500)


def get_param_value(idx):
    global Kp_line, Kd_line, Kp_between, Kd_between
    global BASE_MAX_line, BASE_MAX_between
    global P_FULL_ERROR_LINE, P_FULL_ERROR_BETWEEN
    global BETWEEN_DEADBAND, CORNER_BLACK_THR, TURN_CORNER_MS
    if idx == 0:
        return Kp_line
    if idx == 1:
        return Kd_line
    if idx == 2:
        return Kp_between
    if idx == 3:
        return Kd_between
    if idx == 4:
        return BASE_MAX_line
    if idx == 5:
        return BASE_MAX_between
    if idx == 6:
        return P_FULL_ERROR_LINE
    if idx == 7:
        return P_FULL_ERROR_BETWEEN
    if idx == 8:
        return BETWEEN_DEADBAND
    if idx == 9:
        return CORNER_BLACK_THR
    if idx == 10:
        return TURN_CORNER_MS
    return 0


def update_display_scroll(p, mode, selected_idx):
    display.fill(0)
    if mode == MODE_WAIT:
        state = "WAIT 1st bump"
    elif mode == MODE_LINE:
        state = "MODE: LINE"
    else:
        state = "MODE: BETWEEN"
    display.text(state, 0, 0)
    display.text("A:Next B:- C:+", 0, 10)

    labels = ["KpL", "KdL", "KpB", "KdB",
              "BLm", "BBm", "PFL", "PFB",
              "DBt", "CBT", "TCM"]

    if selected_idx <= 1:
        start = 0
    elif selected_idx >= PARAM_COUNT - 2:
        start = max(0, PARAM_COUNT - 4)
    else:
        start = selected_idx - 1

    row_y = 22
    for i in range(start, min(start + 4, PARAM_COUNT)):
        prefix = ">" if i == selected_idx else " "
        val = get_param_value(i)
        if i <= 3:
            txt = "{}{}:{:.2f}".format(prefix, labels[i], float(val))
        else:
            txt = "{}{}:{}".format(prefix, labels[i], int(val))
        display.text(txt, 0, row_y)
        row_y += 10

    display.show()


def push_history(line):
    global hist_idx, hist_count
    h = history[hist_idx]
    h[0] = line[0]
    h[1] = line[1]
    h[2] = line[2]
    h[3] = line[3]
    h[4] = line[4]
    hist_idx += 1
    if hist_idx >= HISTORY_LEN:
        hist_idx = 0
    if hist_count < HISTORY_LEN:
        hist_count += 1


def corner_direction():
    global hist_idx, hist_count
    if hist_count == 0:
        return 1
    left_total = 0
    right_total = 0
    idx = hist_idx - 1
    if idx < 0:
        idx = HISTORY_LEN - 1
    used = 0
    while used < CORNER_LOOKBACK and used < hist_count:
        h = history[idx]
        left_total += h[0] + h[1]
        right_total += h[3] + h[4]
        idx -= 1
        if idx < 0:
            idx = HISTORY_LEN - 1
        used += 1
    if left_total < right_total:
        return -1
    else:
        return 1


def main():
    global Kp_line, Kd_line, Kp_between, Kd_between
    global BASE_MAX_line, BASE_MAX_between
    global P_FULL_ERROR_LINE, P_FULL_ERROR_BETWEEN
    global BETWEEN_DEADBAND, CORNER_BLACK_THR, TURN_CORNER_MS
    global selected_param_idx
    global last_a_ms, last_b_ms, last_c_ms
    global mode, bump_was_pressed
    global last_loop_ms, dt_nom_ms
    global corner_high_count

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

        if any_pressed and not bump_was_pressed:
            if mode == MODE_WAIT:
                mode = MODE_LINE
                last_p = 0
                p = 0
                last_l = 2000
            elif mode == MODE_LINE:
                mode = MODE_BETWEEN
                last_p = 0
                p = 0
        bump_was_pressed = any_pressed

        line = line_sensors.read_calibrated()
        line_sensors.start_read()
        push_history(line)

        if mode == MODE_BETWEEN:
            all_high = True
            for v in line:
                if v < CORNER_BLACK_THR:
                    all_high = False
                    break
            if all_high:
                corner_high_count += 1
            else:
                corner_high_count = 0

            if corner_high_count >= CORNER_DEBOUNCE:
                d = corner_direction()
                motors.off()
                time.sleep_ms(50)
                if d > 0:
                    motors.set_speeds(BASE_MAX_between, -BASE_MAX_between)
                else:
                    motors.set_speeds(-BASE_MAX_between, BASE_MAX_between)
                time.sleep_ms(TURN_CORNER_MS)
                motors.off()
                corner_high_count = 0
                last_p = 0
                p = 0
                last_l = 2000
                continue

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
            p_full = P_FULL_ERROR_LINE
        elif mode == MODE_BETWEEN:
            p_raw = 2000 - l
            if abs(p_raw) < BETWEEN_DEADBAND:
                p_raw = 0
            Kp_use = Kp_between
            Kd_use = Kd_between
            BASE_MAX_use = BASE_MAX_between
            p_full = P_FULL_ERROR_BETWEEN
        else:
            p_raw = 0
            Kp_use = 0.0
            Kd_use = 0.0
            BASE_MAX_use = 0
            p_full = P_FULL_ERROR_LINE

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
        if abs_p >= p_full:
            base = base_min
        else:
            base = BASE_MAX_use - (BASE_MAX_use - base_min) * abs_p / p_full
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
                selected_param_idx = (selected_param_idx + 1) % PARAM_COUNT
                last_a_ms = now_ms

        if button_b.check():
            if time.ticks_diff(now_ms, last_b_ms) > DEBOUNCE_MS:
                if selected_param_idx == 0:
                    Kp_line = max(0.0, Kp_line - KP_STEP)
                elif selected_param_idx == 1:
                    Kd_line = max(0.0, Kd_line - KD_STEP)
                elif selected_param_idx == 2:
                    Kp_between = max(0.0, Kp_between - KP_STEP)
                elif selected_param_idx == 3:
                    Kd_between = max(0.0, Kd_between - KD_STEP)
                elif selected_param_idx == 4:
                    BASE_MAX_line = max(0, BASE_MAX_line - BASE_STEP)
                elif selected_param_idx == 5:
                    BASE_MAX_between = max(0, BASE_MAX_between - BASE_STEP)
                elif selected_param_idx == 6:
                    P_FULL_ERROR_LINE = max(50, P_FULL_ERROR_LINE - P_FULL_STEP)
                elif selected_param_idx == 7:
                    P_FULL_ERROR_BETWEEN = max(50, P_FULL_ERROR_BETWEEN - P_FULL_STEP)
                elif selected_param_idx == 8:
                    BETWEEN_DEADBAND = max(0, BETWEEN_DEADBAND - DEADBAND_STEP)
                elif selected_param_idx == 9:
                    CORNER_BLACK_THR = max(100, CORNER_BLACK_THR - CORNER_THR_STEP)
                elif selected_param_idx == 10:
                    TURN_CORNER_MS = max(50, TURN_CORNER_MS - TURN_MS_STEP)
                last_b_ms = now_ms

        if button_c.check():
            if time.ticks_diff(now_ms, last_c_ms) > DEBOUNCE_MS:
                if selected_param_idx == 0:
                    Kp_line += KP_STEP
                elif selected_param_idx == 1:
                    Kd_line += KD_STEP
                elif selected_param_idx == 2:
                    Kp_between += KP_STEP
                elif selected_param_idx == 3:
                    Kd_between += KD_STEP
                elif selected_param_idx == 4:
                    BASE_MAX_line = min(max_speed, BASE_MAX_line + BASE_STEP)
                elif selected_param_idx == 5:
                    BASE_MAX_between = min(max_speed, BASE_MAX_between + BASE_STEP)
                elif selected_param_idx == 6:
                    P_FULL_ERROR_LINE += P_FULL_STEP
                elif selected_param_idx == 7:
                    P_FULL_ERROR_BETWEEN += P_FULL_STEP
                elif selected_param_idx == 8:
                    BETWEEN_DEADBAND += DEADBAND_STEP
                elif selected_param_idx == 9:
                    CORNER_BLACK_THR = min(1000, CORNER_BLACK_THR + CORNER_THR_STEP)
                elif selected_param_idx == 10:
                    TURN_CORNER_MS += TURN_MS_STEP
                last_c_ms = now_ms

        if (now_ms % 100) < 10:
            update_display_scroll(p, mode, selected_param_idx)

        time.sleep_ms(5)


main()
