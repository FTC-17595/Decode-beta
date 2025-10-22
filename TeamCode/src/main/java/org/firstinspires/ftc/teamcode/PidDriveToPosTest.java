package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Pinpoint Drive to Position")
public class PidDriveToPosTest extends LinearOpMode {

    // Hardware members
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;

    // PID coefficients for X, Y, and Heading, please tune these
    private final double Kp_x = 0.02;
    private final double Ki_x = 0.0001;
    private final double Kd_x = 0.002;
    private double integralSum_x = 0;
    private double lastError_x = 0;

    private final double Kp_y = 0.02;
    private final double Ki_y = 0.0001;
    private final double Kd_y = 0.002;
    private double integralSum_y = 0;
    private double lastError_y = 0;

    private final double Kp_h = 0.02;
    private final double Ki_h = 0.0001;
    private final double Kd_h = 0.002;
    private double integralSum_h = 0;
    private double lastError_h = 0;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Map hardware
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set motor directions (adjust based on your robot)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Target coordinates and heading
        driveToPos(1000, 1000, 90); // Example: move to (1000, 1000) and turn to 90 degrees
    }

    private void driveToPos(double targetX, double targetY, double targetHeading) {
        // Reset timers and PID variables
        timer.reset();
        integralSum_x = 0;
        lastError_x = 0;
        integralSum_y = 0;
        lastError_y = 0;
        integralSum_h = 0;
        lastError_h = 0;

        // Loop until robot is near target
        while (opModeIsActive() && !isAtTarget(targetX, targetY, targetHeading)) {
            // Get current pose from Pinpoint
            Pose2D currentPose = pinpoint.getPosition();
            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);
            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);

            // Calculate errors
            double error_x = targetX - currentX;
            double error_y = targetY - currentY;
            double error_h = targetHeading - currentHeading;

            // PID Calculations
            double output_x = pidCalculate(Kp_x, Ki_x, Kd_x, error_x, lastError_x, integralSum_x, timer);
            double output_y = pidCalculate(Kp_y, Ki_y, Kd_y, error_y, lastError_y, integralSum_y, timer);
            double output_h = pidCalculate(Kp_h, Ki_h, Kd_h, error_h, lastError_h, integralSum_h, timer);

            lastError_x = error_x;
            lastError_y = error_y;
            lastError_h = error_h;

            // Combine PID outputs for mecanum drive
            double frontLeftPower = output_x + output_y + output_h;
            double backLeftPower = output_x - output_y + output_h;
            double frontRightPower = output_x - output_y - output_h;
            double backRightPower = output_x + output_y - output_h;

            // Normalize motor powers to keep within range [-1, 1]
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            telemetry.addData("X", currentX);
            telemetry.addData("Y", currentY);
            telemetry.addData("Heading", currentHeading);
            telemetry.update();

            timer.reset();
        }

        // Stop the robot
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // Helper method for PID calculation
    private double pidCalculate(double Kp, double Ki, double Kd, double error, double lastError, double integralSum, ElapsedTime timer) {
        double derivative = (error - lastError) / timer.seconds();
        integralSum += (error * timer.seconds());
        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    // Helper method to check if robot is at target position and heading
    private boolean isAtTarget(double targetX, double targetY, double targetHeading) {
        Pose2D currentPose = pinpoint.getPosition();
        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);
        double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);

        double x_diff = Math.abs(targetX - currentX);
        double y_diff = Math.abs(targetY - currentY);
        double heading_diff = Math.abs(targetHeading - currentHeading);

        double distance = Math.hypot(x_diff, y_diff);

        // Define a small tolerance for the target position and heading
        final double DISTANCE_TOLERANCE = 20.0; // in mm
        final double HEADING_TOLERANCE = 2.0; // in degrees

        return (distance < DISTANCE_TOLERANCE) && (heading_diff < HEADING_TOLERANCE);
    }
}

