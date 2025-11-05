package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DecodeOdo  {
    LinearOpMode linearOpMode;
    GoBildaPinpointDriver odo;
    IMU imu;

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DecodeOdo(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        // Hardware initialization
        frontLeftMotor  = linearOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        frontRightMotor = linearOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        backLeftMotor   = linearOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor  = linearOpMode.hardwareMap.dcMotor.get("backRightMotor");

        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
        odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

    public void displayTelemetryAuto() {
        linearOpMode.telemetry.addData("x: ", odo.getPosX(DistanceUnit.MM));
        linearOpMode.telemetry.addData("y: ", odo.getPosY(DistanceUnit.MM));
    }

    public final boolean opModeIsActive() {
        boolean isActive = !this.linearOpMode.isStopRequested() && this.linearOpMode.isStarted();
        if (isActive) {
            linearOpMode.idle();
        }
        return isActive;
    }
    public void driveToPos(double targetX, double targetY) {
        // Update odometry before starting
        odo.update();

        boolean telemAdded = false;  // Flag so telemetry is printed only once

        // Keep driving while opmode is active AND the robot is more than 30 cm away in X or Y
        while (opModeIsActive() &&
                (Math.abs(targetX - odo.getPosX(DistanceUnit.MM)) > 30 ||
                        Math.abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

            // Update odometry each loop to get the latest position
            odo.update();

            // Compute distance from target in X and Y, scaled down for motor power
            // The 0.001 factor converts cm error into a smaller motor power signal
            // Negative Y compensates for coordinate orientation differences
            double x = 0.0001 * (targetX - odo.getPosX(DistanceUnit.MM));
            double y = -0.0001 * (targetY - odo.getPosY(DistanceUnit.MM));

            // Get the robot's heading (rotation angle) from the IMU in radians
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the field-relative (x, y) error into robot-relative coordinates
            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            // Add telemetry only once to avoid spamming output
            if (!telemAdded) {
                linearOpMode.telemetry.addData("x: ", x);
                linearOpMode.telemetry.addData("y: ", y);
                linearOpMode.telemetry.addData("rotX: ", rotX);
                linearOpMode.telemetry.addData("rotY: ", rotY);

                telemAdded = true;
            }

            // Enforce a minimum power threshold so the robot doesn't stall
            if (Math.abs(rotX) < 0.15) {
                rotX = Math.signum(rotX) * 0.15;
            }
            if (Math.abs(rotY) < 0.15) {
                rotY = Math.signum(rotY) * 0.15;
            }

            // Normalize powers to keep them within [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

            // Calculate motor powers for a simple tank-style drivetrain
            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            // Apply the calculated motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Optionally, you could update telemetry here for debugging

        }

        // Stop all motors when target position is reached or opmode ends
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    public void gyroTurnToAngle(double turnAngle) {
        double error, currentHeadingAngle, driveMotorsPower;
        imu.resetYaw();

        error = turnAngle;

        while (opModeIsActive() && ((error > 1) || (error < -1))) {
            odo.update();
            linearOpMode.telemetry.addData("X: ", odo.getPosX(DistanceUnit.CM));
            linearOpMode.telemetry.addData("Y: ", odo.getPosY(DistanceUnit.CM));
//                telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            linearOpMode.telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            linearOpMode.telemetry.update();

                /*driveMotorsPower = error / 200;

                if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
                    driveMotorsPower = 0.2;
                } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
                    driveMotorsPower = -0.2;
                }*/
            driveMotorsPower = error / 50;

            if ((driveMotorsPower < 0.35) && (driveMotorsPower > 0)) {
                driveMotorsPower = 0.35;
            } else if ((driveMotorsPower > -0.35) && (driveMotorsPower < 0)) {
                driveMotorsPower = -0.35;
            }
            // Positive power causes left turn
            frontLeftMotor.setPower(-driveMotorsPower);
            backLeftMotor.setPower(-driveMotorsPower);
            frontRightMotor.setPower(driveMotorsPower);
            backRightMotor.setPower(driveMotorsPower);

            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);


    }


}