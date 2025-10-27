package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private final DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private final IMU imu;

    private final LinearOpMode linearOpMode;
    private int counter = 0;

    private double ROTATE_SPEED_ADJUSTER = TeleOpConstants.RIGHT_JOYSTICK_SPEED_ADJUSTER;
    private double DRIVE_AND_STRAFE_SPEED_ADJUSTER = TeleOpConstants.LEFT_JOYSTICK_SPEED_ADJUSTER;

    public DriveTrain(LinearOpMode linearOpMode) {
        frontLeftMotor = linearOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = linearOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor = linearOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = linearOpMode.hardwareMap.dcMotor.get("backRightMotor");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));

        this.linearOpMode = linearOpMode;
    }

    public void configureMotorModes() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu.resetYaw();
    }

    static class Powers {
        private final double frontLeftPower;
        private final double frontRightPower;
        private final double backLeftPower;
        private final double backRightPower;

        public Powers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
            this.frontLeftPower = frontLeftPower / TeleOpConstants.LEFT_JOYSTICK_SPEED_ADJUSTER;
            this.frontRightPower = frontRightPower / TeleOpConstants.LEFT_JOYSTICK_SPEED_ADJUSTER;
            this.backLeftPower = backLeftPower / TeleOpConstants.LEFT_JOYSTICK_SPEED_ADJUSTER;
            this.backRightPower = backRightPower / TeleOpConstants.LEFT_JOYSTICK_SPEED_ADJUSTER;
        }

        public double getFrontLeftPower() {
            return frontLeftPower;
        }

        public double getFrontRightPower() {
            return frontRightPower;
        }

        public double getBackLeftPower() {
            return backLeftPower;
        }

        public double getBackRightPower() {
            return backRightPower;
        }
    }

    static class Rotation {
        private final double rotX;
        private final double rotY;

        public Rotation(double rotX, double rotY) {
            this.rotX = rotX;
            this.rotY = rotY;
        }

        public double getRotX() {
            return rotX;
        }

        public double getRotY() {
            return rotY;
        }
    }

    private Powers getPowers(double y, double x, double rx) {
        // Get the rotation values
        Rotation rotation = getRotation(x, y);
        double rotX = rotation.getRotX();
        double rotY = rotation.getRotY();

        // Calculate the denominator
        double denominator = getDenominator(rotX, rotY, rx);

        // Calculate each wheel power using the rotation values
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        return new Powers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private double getDenominator(double rotX, double rotY, double rx) {
        // Calculate the denominator to normalize powers
        return Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    }

    private Rotation getRotation(double x, double y) {
        // Calculate the robot's heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate rotated x and y values
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Apply scaling to rotX if needed
        rotX = rotX * 1.1;

        // Return a Rotation object containing rotX and rotY
        return new Rotation(rotX, rotY);
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Left Stick (X, Y)", "%5.2f, %5.2f", linearOpMode.gamepad1.left_stick_x / DRIVE_AND_STRAFE_SPEED_ADJUSTER, -linearOpMode.gamepad1.left_stick_y / DRIVE_AND_STRAFE_SPEED_ADJUSTER);
        linearOpMode.telemetry.addData("Right Stick (Rotation)", "%5.2f", linearOpMode.gamepad1.right_stick_x / ROTATE_SPEED_ADJUSTER);
        linearOpMode.telemetry.addData("Raw Left Stick (X, Y)", "%5.2f, %5.2f", linearOpMode.gamepad1.left_stick_x, -linearOpMode.gamepad1.left_stick_y);
        linearOpMode.telemetry.addData("Raw Right Stick (Rotation)", "%5.2f", linearOpMode.gamepad1.right_stick_x);
        linearOpMode.telemetry.addData("Fast Mode", linearOpMode.gamepad1.left_bumper);
        linearOpMode.telemetry.addData("Slow Mode", linearOpMode.gamepad1.right_bumper);
        linearOpMode.telemetry.addData("Yaw, Pitch, Roll", imu.getRobotYawPitchRollAngles());
        linearOpMode.telemetry.addData("Reset Yaw", linearOpMode.gamepad1.back);

        linearOpMode.telemetry.addData("Front Left Position", frontLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Front Right Position", frontRightMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Back Left Position", backLeftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Back Right Position", backRightMotor.getCurrentPosition());

        linearOpMode.telemetry.addData("Counter", counter);
    }

    public void setMotorPowers() {
        if (linearOpMode.gamepad1.start) {
            return;
        }

        Powers motorPower = getPowers(-linearOpMode.gamepad1.left_stick_y / DRIVE_AND_STRAFE_SPEED_ADJUSTER, linearOpMode.gamepad1.left_stick_x / DRIVE_AND_STRAFE_SPEED_ADJUSTER, linearOpMode.gamepad1.right_stick_x / ROTATE_SPEED_ADJUSTER);
        frontLeftMotor.setPower(motorPower.getFrontLeftPower());
        frontRightMotor.setPower(motorPower.getFrontRightPower());
        backLeftMotor.setPower(motorPower.getBackLeftPower());
        backRightMotor.setPower(motorPower.getBackRightPower());
    }

    public void adjustTurnSpeed() {
        if (linearOpMode.gamepad1.right_bumper) {
            ROTATE_SPEED_ADJUSTER = 3;
            DRIVE_AND_STRAFE_SPEED_ADJUSTER = 3;
        }

        if (linearOpMode.gamepad1.left_bumper) {
            ROTATE_SPEED_ADJUSTER = 1;
            DRIVE_AND_STRAFE_SPEED_ADJUSTER = 1;
        }
    }

    public void resetYaw() {
        if (linearOpMode.gamepad1.start) {
            imu.resetYaw();
            counter++;
        }
    }

    private void strafeLeft() {
        if (frontLeftMotor.getPower() != 0) {
            return;
        }

        double power = 0.6;
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(-power);
        backLeftMotor.setPower(-power);
        backRightMotor.setPower(power);
    }

    private void strafeRight() {
        if (frontLeftMotor.getPower() != 0) {
            return;
        }

        double power = 0.6;
        frontLeftMotor.setPower(-power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(-power);
    }

    public void setMovePower(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }
}