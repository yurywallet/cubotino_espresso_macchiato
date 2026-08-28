from machine import Pin, PWM
from utime import sleep_ms


TOP_SERVO_PIN = 22
BOTTOM_SERVO_PIN = 23
SERVO_FREQUENCY = 50
# Start with the safe range for a normal 1-2 ms servo.
# Use 20 and 134 only when the servo datasheet confirms a 0.5-2.5 ms range.
SERVO_MIN_DUTY = 48       # Approximately 1.0 ms at 50 Hz.
SERVO_MAX_DUTY = 106      # Approximately 2.0 ms at 50 Hz.
# Project reference midpoint: matches b_home and t_servo_close in
# Cubotino_settings.txt. At 50 Hz this is approximately a 1.5 ms pulse.
SERVO_MID_DUTY = 76
POSITION_HOLD_MS = 2000
FINAL_SETTLE_MS = 5000
CALIBRATION_CYCLES = 2


def set_servo_position(servo, duty):
    """Set one servo to a PWM duty value."""
    servo.duty(duty)


def move_and_hold(name, servo, duty):
    """Move to a position and hold it long enough for visual confirmation."""
    set_servo_position(servo, duty)
    print("{} servo: holding duty {} for visual check.".format(name, duty))
    sleep_ms(POSITION_HOLD_MS)


def calibrate_servo(name, servo):
    """Sweep one servo and leave it at the project reference midpoint."""
    print("\n--- {} servo: initial position check ---".format(name))
    move_and_hold(name, servo, SERVO_MID_DUTY)
    print("{} servo commanded to project midpoint duty {}.".format(
        name, SERVO_MID_DUTY))
    print("If it did not move or hold, stop and check power, ground, signal pin, and horn.")

    for cycle in range(1, CALIBRATION_CYCLES + 1):
        print("{} servo: calibration cycle {}/{}".format(
            name, cycle, CALIBRATION_CYCLES))

        print("{} servo: moving to first limit.".format(name))
        move_and_hold(name, servo, SERVO_MIN_DUTY)

        print("{} servo: moving to second limit.".format(name))
        move_and_hold(name, servo, SERVO_MAX_DUTY)

        print("{} servo: returning to midpoint duty {}.".format(
            name, SERVO_MID_DUTY))
        move_and_hold(name, servo, SERVO_MID_DUTY)
        print("{} servo: midpoint reached for cycle {}.".format(name, cycle))

    print("{} servo: final midpoint command sent.".format(name))
    print("{} servo: waiting {} seconds for the mechanism to settle.".format(
        name, FINAL_SETTLE_MS // 1000))
    sleep_ms(FINAL_SETTLE_MS)
    print("{} servo: calibration command complete; inspect that it is stopped.".format(name))
    print("Fit or adjust this servo horn while duty {} is active.".format(
        SERVO_MID_DUTY))


def swipe_and_center():
    """Sweep both servos and leave them at their electrical midpoint.

    Remove the servo horns before running this procedure. Install them while
    the servos are holding SERVO_MID_DUTY at the end of the procedure.
    """
    top_servo = PWM(Pin(TOP_SERVO_PIN), freq=SERVO_FREQUENCY)
    sleep_ms(50)
    bottom_servo = PWM(Pin(BOTTOM_SERVO_PIN), freq=SERVO_FREQUENCY)
    sleep_ms(50)

    try:
        calibrate_servo("Top", top_servo)
        calibrate_servo("Bottom", bottom_servo)
        print("\n=== Calibration complete for both servos ===")
        print("Both servos are holding midpoint duty {}; inspect before powering off.".format(
            SERVO_MID_DUTY))
    except KeyboardInterrupt:
        set_servo_position(top_servo, SERVO_MID_DUTY)
        set_servo_position(bottom_servo, SERVO_MID_DUTY)
        print("\nCalibration interrupted; both servos returned to midpoint duty {}.".format(
            SERVO_MID_DUTY))



if __name__ == "__main__":
    swipe_and_center()