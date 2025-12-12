package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoConstants {

    private LinearOpMode linearOpMode;
    GoBildaPinpointDriver odo;
    static final double ARTIFACT_PICKUP_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.5;
    static final double LAUNCH_EXIT_SPEED = 0.8;
    static final double SHOOT_X = 1437;
    static final double SHOOT_Y = -307.373;
    static final double AUTO_ARTIFACT_SHOOT_POWER = 0.72;
    static final double AUTO_STRAFE_POWER = 0.5;
    final static double SHORT_RANGE_VELOCITY =1238;
    final static double LONG_RANGE_VELOCITY = 1255; // prev: 1275
//    final static double LONG_RANGE_VELOCITY = 1500; // Please don't edit this part
    final static double MAX_VELOCITY = 2300;
    final static  double kP = 85.0;
    final static  double kI = 0.00;
    final static double kD = 26;
    final static  double kF = 32767 / MAX_VELOCITY * 0.88;
    final static double SHOOTING_SPINUP_TIME = 2000;
    final static double FLAP_SLEEP = 800;
    final static long FEED_TIME_AUTO = 900;

}
