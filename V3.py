

from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()

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

Kp = 3.0     
Kd = 30.0        
BASE_MAX = 2700 #max_speed // 3 

# Steps for tuning
KP_STEP   = 0.1
KD_STEP   = 0.5
BASE_STEP = 100


selected_param = 0

DEBOUNCE_MS = 200
last_a_ms = 0
last_b_ms = 0
last_c_ms = 0

LINE_TOTAL_MIN = 200   
# tune for better 90 degree turns   
P_FULL_ERROR   = 1200     
BASE_MIN_FRACTION = 0.2  


def calibrate():
    display.fill(0)
    display.text("Calibrating", 0, 0)
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
    display.text("PID Tuner", 0, 0)
    display.text("A:Sel  B:-  C:+", 0, 10)
    display.show()
    time.sleep_ms(500)



def update_display(p, Kp, Kd, BASE_MAX, selected_param):
    display.fill(0)
    display.text("PID Tuner", 0, 0)
    display.text("A:Sel B:- C:+", 0, 10)

    # Mark selected parameter with ">"
    if selected_param == 0:
        prefix_kp = ">"
        prefix_kd = " "
        prefix_bs = " "
    elif selected_param == 1:
        prefix_kp = " "
        prefix_kd = ">"
        prefix_bs = " "
    else:
        prefix_kp = " "
        prefix_kd = " "
        prefix_bs = ">"

    display.text("{}Kp:{:.2f}".format(prefix_kp, Kp), 0, 22)
    display.text("{}Kd:{:.2f}".format(prefix_kd, Kd), 0, 32)
    display.text("{}BaseMax:{}".format(prefix_bs, int(BASE_MAX)), 0, 42)

    display.text("p={}".format(int(p)), 0, 54)

    display.show()


def main():
    global Kp, Kd, BASE_MAX, selected_param
    global last_a_ms, last_b_ms, last_c_ms

    calibrate()

    last_p = 0
    p = 0
    last_l = 2000  


    line_sensors.start_read()

    while True:
        now_ms = time.ticks_ms()

        line = line_sensors.read_calibrated()
        line_sensors.start_read()

        total = line[0] + line[1] + line[2] + line[3] + line[4]

        if total > LINE_TOTAL_MIN:
            weighted_sum = (
                0    * line[0] +
                1000 * line[1] +
                2000 * line[2] +
                3000 * line[3] +
                4000 * line[4]
            )
            l = weighted_sum // total
            last_l = l
        else:
           
            l = last_l

        
        p = l - 2000        
        d = p - last_p      
        last_p = p


        base_min = int(BASE_MAX * BASE_MIN_FRACTION)
        if base_min < 0:
            base_min = 0
        if base_min > BASE_MAX:
            base_min = BASE_MAX

        abs_p = abs(p)
        if abs_p >= P_FULL_ERROR:
            base = base_min
        else:

            base = BASE_MAX - (BASE_MAX - base_min) * abs_p / P_FULL_ERROR
            base = int(base)

   
        turn = Kp * p + Kd * d

        if turn > base:
            turn = base
        elif turn < -base:
            turn = -base

        left = base + int(turn)
        right = base - int(turn)

        if left < 0: left = 0
        if left > max_speed: left = max_speed
        if right < 0: right = 0
        if right > max_speed: right = max_speed

        motors.set_speeds(left, right)

     
        if button_a.check():
            if time.ticks_diff(now_ms, last_a_ms) > DEBOUNCE_MS:
                selected_param = (selected_param + 1) % 3
                last_a_ms = now_ms

        
        if button_b.check():
            if time.ticks_diff(now_ms, last_b_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    Kp = max(0.0, Kp - KP_STEP)
                elif selected_param == 1:
                    Kd = max(0.0, Kd - KD_STEP)
                else:
                    BASE_MAX = max(0, BASE_MAX - BASE_STEP)
                last_b_ms = now_ms

        
        if button_c.check():
            if time.ticks_diff(now_ms, last_c_ms) > DEBOUNCE_MS:
                if selected_param == 0:
                    Kp += KP_STEP
                elif selected_param == 1:
                    Kd += KD_STEP
                else:
                    BASE_MAX = min(max_speed, BASE_MAX + BASE_STEP)
                last_c_ms = now_ms

   
        if (now_ms % 100) < 10:
            update_display(p, Kp, Kd, BASE_MAX, selected_param)

        time.sleep_ms(5)  

main()
