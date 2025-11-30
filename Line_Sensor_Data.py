from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
button_a = robot.ButtonA()

edition = editions.select()
if edition == "Standard":
    calibration_speed = 1000
    calibration_count = 100
elif edition == "Turtle":
    calibration_speed = 3000
    calibration_count = 100
elif edition == "Hyper":
    calibration_speed = 1000
    calibration_count = 100
    motors.flip_left(True)
    motors.flip_right(True)

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

calibrate()

while True:
    if button_a.check():
        break

    vals = line_sensors.read_calibrated()

    display.fill(0)
    display.text("L0 {} ".format(int(vals[0])), 0, 0)
    display.text("L1 {} ".format(int(vals[1])), 0, 10)
    display.text("L2 {} ".format(int(vals[2])), 0, 20)
    display.text("L3 {} ".format(int(vals[3])), 0, 30)
    display.text("L4 {} ".format(int(vals[4])), 0, 40)
    display.show()

    time.sleep_ms(50)
