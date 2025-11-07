package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Auto Red Alliance", group = "Odometry")
public class AutoMoveAndOuttakePreloads extends LinearOpMode {

    private ArtifactHandlingSystem artifactHandlingSystem;
    private DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private DcMotorEx leftOdom, rightOdom, strafeOdom;
    private DcMotorEx IntakePower;

    private static final double ODOM_TICKS_PER_INCH = 8192.0 / (Math.PI * 2);
    private static final double ODOM_TRACK_WIDTH = 14.0;
    private static final double ODOM_CENTER_OFFSET = 7.0;

    private double xPos = 0, yPos = 0, heading = 0;
    private int lastLeftPos = 0, lastRightPos = 0, lastStrafePos = 0;

    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        leftOdom = hardwareMap.get(DcMotorEx.class, "leftOdom");
        rightOdom = hardwareMap.get(DcMotorEx.class, "rightOdom");
        strafeOdom = hardwareMap.get(DcMotorEx.class, "strafeOdom");
        IntakePower = hardwareMap.get(DcMotorEx.class, "IntakePower");

        resetEncoders(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftOdom, rightOdom, strafeOdom);
        setRunWithoutEncoder(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, leftOdom, rightOdom, strafeOdom);

        artifactHandlingSystem = new ArtifactHandlingSystem(this);
        artifactHandlingSystem.configureMotorModes();
        artifactHandlingSystem.flapSystem(false);
        artifactHandlingSystem.intakeSystem(false, false);

        telemetry.addLine("Ready to start autonomous");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        turnToHeading(22, 0.75);
        artifactHandlingSystem.outtakeSystem(true, 0.5);
        sleep(7000);
        artifactHandlingSystem.outtakeSystem(false, 0);
        turnToHeading(90, 0.75);
        strafeForward(120, 0.6);
        moveRight(60, 0.6, true);
        artifactHandlingSystem.intakeSystem(false, false);
        moveBack(60, 0.6);
        turnToHeading(68, 0.75);
        artifactHandlingSystem.outtakeSystem(true, 0.5);
        sleep(7000);
        artifactHandlingSystem.outtakeSystem(false, 0);
    }

    private void updateOdometry() {
        int leftPos = leftOdom.getCurrentPosition();
        int rightPos = rightOdom.getCurrentPosition();
        int strafePos = strafeOdom.getCurrentPosition();
        int dL = leftPos - lastLeftPos;
        int dR = rightPos - lastRightPos;
        int dS = strafePos - lastStrafePos;
        lastLeftPos = leftPos;
        lastRightPos = rightPos;
        lastStrafePos = strafePos;
        double dLInch = dL / ODOM_TICKS_PER_INCH;
        double dRInch = dR / ODOM_TICKS_PER_INCH;
        double dSInch = dS / ODOM_TICKS_PER_INCH;
        double dTheta = (dRInch - dLInch) / ODOM_TRACK_WIDTH;
        double avgTheta = heading + (dTheta / 2.0);
        double dx = dSInch - (dTheta * ODOM_CENTER_OFFSET);
        double dy = (dLInch + dRInch) / 2.0;
        xPos += dx * Math.cos(avgTheta) - dy * Math.sin(avgTheta);
        yPos += dx * Math.sin(avgTheta) + dy * Math.cos(avgTheta);
        heading += dTheta;
    }

    private void turnToHeading(double targetDeg, double power) {
        double targetRad = Math.toRadians(targetDeg);
        while (opModeIsActive() && !isStopRequested()) {
            updateOdometry();
            double error = targetRad - heading;
            if (Math.abs(error) < Math.toRadians(2)) break;
            double turnPower = Math.signum(error) * power;
            frontLeftMotor.setPower(turnPower);
            backLeftMotor.setPower(turnPower);
            frontRightMotor.setPower(-turnPower);
            backRightMotor.setPower(-turnPower);
            telemetry.addData("Heading", Math.toDegrees(heading));
            telemetry.update();
            idle();
        }
        stopMotors();
    }

    private void strafeForward(double distanceInches, double power) {
        double targetX = xPos;
        double targetY = yPos + distanceInches;
        goToPosition(targetX, targetY, power);
    }

    private void moveRight(double distanceInches, double power, boolean startIntake) {
        if (startIntake) artifactHandlingSystem.intakeSystem(true, true);
        double targetX = xPos + distanceInches;
        double targetY = yPos;
        goToPosition(targetX, targetY, power);
    }

    private void moveBack(double distanceInches, double power) {
        double targetX = xPos;
        double targetY = yPos - distanceInches;
        goToPosition(targetX, targetY, power);
    }

    private void goToPosition(double targetX, double targetY, double power) {
        while (opModeIsActive() && !isStopRequested()) {
            updateOdometry();
            double dx = targetX - xPos;
            double dy = targetY - yPos;
            double distance = Math.hypot(dx, dy);
            if (distance < 1) break;
            double angle = Math.atan2(dy, dx);
            double flPower = power * (Math.sin(angle - heading) + Math.cos(angle - heading));
            double frPower = power * (Math.sin(angle - heading) - Math.cos(angle - heading));
            double blPower = power * (Math.sin(angle - heading) - Math.cos(angle - heading));
            double brPower = power * (Math.sin(angle - heading) + Math.cos(angle - heading));
            frontLeftMotor.setPower(flPower);
            frontRightMotor.setPower(frPower);
            backLeftMotor.setPower(blPower);
            backRightMotor.setPower(brPower);
            telemetry.addData("x", xPos);
            telemetry.addData("y", yPos);
            telemetry.update();
            idle();
        }
        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void resetEncoders(DcMotorEx... motors) {
        for (DcMotorEx m : motors) m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunWithoutEncoder(DcMotorEx... motors) {
        for (DcMotorEx m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
