package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.pathgen.Point;

@Config
public class PedroConstants {

    public static double MASS = 12.5; 

    public static double X_OFFSET = 2.56; 
    public static double Y_OFFSET = 5.60; 

    public static double TRACK_WIDTH = 14.0; 
    public static double WHEEL_BASE = 14.0; 

    public static double MAX_VELOCITY = 70.0;
    public static double MAX_ACCELERATION = 60.0;
    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(180);
    public static double MAX_ANGULAR_ACCELERATION = Math.toRadians(90);

    public static double xMovementP = 0.06;
    public static double xMovementD = 0.01;
    
    public static double yMovementP = 0.06;
    public static double yMovementD = 0.01;
    
    public static double headingP = 0.3;
    public static double headingD = 0.02;
    
    public static double driveP = 0.02; 
}
