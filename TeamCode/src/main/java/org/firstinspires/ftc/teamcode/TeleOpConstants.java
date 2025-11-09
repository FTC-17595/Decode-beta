package org.firstinspires.ftc.teamcode;

public class TeleOpConstants {
    final static double LEFT_JOYSTICK_SPEED_ADJUSTER = 1.0;
    final static double RIGHT_JOYSTICK_SPEED_ADJUSTER = 1.0;
    final static double FLAP_SERVO_UP = 1.0;
    final static double FLAP_SERVO_DOWN = 0.67;
    final static double BLANK = 0.000;
    final static double GREEN = 0.500;
    final static double VIOLET = 0.720;
    final static double RED = 0.333;
    final static double BLUE = 0.611;
    final static double SHORT_RANGE_VELOCITY = 1250;
    final static double LONG_RANGE_VELOCITY = 1500;
    final static double AUTO_SHORT_RANGE_VELOCITY = 950;
    final static double AUTO_LONG_RANGE_VELOCITY = 1250;
    final static double MAX_VELOCITY = 2300.0;
    final static double VELOCITY_INCREMENT = 10.0;
    final static long SPINUP_MS = 2000;
    final static long SERVO_UP_MS = 250;
    final static long SERVO_RESET_MS = 150;
    final static long FEED_MS = 1000;
    final static long FEED_SETTLE_MS = 150;
    final static long PREFIRE_WAIT_MS = 200;
    final static double CELEBRATION_SPEED = 3500;
    final static int WAIT_FOR_MOTOR_OFFSET = 20;
    final static double kP = 18.0;
    final static double kI = 0.15;
    final static double kD = 7.5;
    final static double kF = 32767 / TeleOpConstants.MAX_VELOCITY * 0.88;
}
