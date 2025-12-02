from pololu_3pi_2040_robot import robot
import time

display = robot.Display()
line_sensors = robot.LineSensors()
bump_sensors = robot.BumpSensors()
button_a = robot.ButtonA()

bump_sensors.calibrate()

display.fill(0)
display.text("Manual Logger", 0, 0)
display.text("Push robot", 0, 10)
display.text("A: stop", 0, 20)
display.show()

f = open("log.csv", "w")
f.write("t_ms,l0,l1,l2,l3,l4,bumpL,bumpR\n")

last_log_ms = time.ticks_ms()
LOG_INTERVAL_MS = 20

last_bL = -1
last_bR = -1

def log_once(t_ms):
    line_raw = line_sensors.read()
    bump_sensors.read()
    bL = 1 if bump_sensors.left_is_pressed() else 0
    bR = 1 if bump_sensors.right_is_pressed() else 0
    l0 = int(line_raw[0])
    l1 = int(line_raw[1])
    l2 = int(line_raw[2])
    l3 = int(line_raw[3])
    l4 = int(line_raw[4])
    f.write("{},{},{},{},{},{},{},{}\n".format(
        t_ms, l0, l1, l2, l3, l4, bL, bR
    ))
    return bL, bR

while True:
    now_ms = time.ticks_ms()

    if button_a.check():
        break

    if time.ticks_diff(now_ms, last_log_ms) >= LOG_INTERVAL_MS:
        last_log_ms = now_ms
        bL, bR = log_once(now_ms)
        if bL != last_bL or bR != last_bR:
            display.fill(0)
            display.text("Logging", 0, 0)
            display.text("A: stop", 0, 10)
            display.text("L:{} R:{}".format(bL, bR), 0, 24)
            display.show()
            last_bL = bL
            last_bR = bR

    time.sleep_ms(5)

f.flush()
f.close()

display.fill(0)
display.text("Done.", 0, 0)
display.text("log.csv saved", 0, 10)
display.show()
