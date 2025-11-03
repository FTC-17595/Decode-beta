package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;


@Autonomous(name = "DriveToTest")
    public class DriveToTest extends LinearOpMode {

        GoBildaPinpointDriver odo;
        DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

        IMU imu;
        AprilTagProcessor tagProcessor;
        @Override
        public void runOpMode() {

            initAuto();
            waitForStart();
            resetRuntime();
            if(isStopRequested()) return;


            PinpointForward(1200);


//            while (opModeIsActive()) {
//                odo.update();
//
//                 /*
//            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
//             */
//                Pose2D pos = odo.getPosition();
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//                telemetry.addData("Position", data);
//                telemetry.addData("Status", odo.getDeviceStatus());
//                telemetry.update();
//
//            }




        }

        // Drives the robot toward a given (X, Y) coordinate using odometry and IMU heading
        private void driveToPos(double targetX, double targetY) {
            // Update odometry before starting
            odo.update();

            boolean telemAdded = false;  // Flag so telemetry is printed only once

            // Keep driving while opmode is active AND the robot is more than 30 cm away in X or Y
            while (opModeIsActive() &&
                    (abs(targetX - odo.getPosX(DistanceUnit.MM)) > 30 ||
                            abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

                // Update odometry each loop to get the latest position
                odo.update();

                // Compute distance from target in X and Y, scaled down for motor power
                // The 0.001 factor converts cm error into a smaller motor power signal
                // Negative Y compensates for coordinate orientation differences
                double x = 0.001 * (targetX - odo.getPosX(DistanceUnit.MM));
                double y = -0.001 * (targetY - odo.getPosY(DistanceUnit.MM));

                // Get the robot's heading (rotation angle) from the IMU in radians
                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the field-relative (x, y) error into robot-relative coordinates
                double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
                double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

                // Add telemetry only once to avoid spamming output
                if (!telemAdded) {
                    telemetry.addData("x: ", x);
                    telemetry.addData("y: ", y);
                    telemetry.addData("rotX: ", rotX);
                    telemetry.addData("rotY: ", rotY);
                    telemetry.update();
                    telemAdded = true;
                }

                // Enforce a minimum power threshold so the robot doesn't stall
                if (abs(rotX) < 0.15) {
                    rotX = Math.signum(rotX) * 0.15;
                }
                if (abs(rotY) < 0.15) {
                    rotY = Math.signum(rotY) * 0.15;
                }

                // Normalize powers to keep them within [-1, 1]
                double denominator = Math.max(abs(y) + abs(x), 1);

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
                telemetry.update();
            }

            // Stop all motors when target position is reached or opmode ends
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }
        private void PinpointForward(double target) {



                double margin = target + odo.getPosX(DistanceUnit.MM);


                while (opModeIsActive() && abs(margin) > 100) {

                    odo.update();

                    Pose2D pos = odo.getPosition();
                    String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
                    telemetry.addData("Position", data);
                    telemetry.addData("Status", odo.getDeviceStatus());

                    double direction = Math.signum(margin);
                    double power = 0.5 * direction;
                    double current = odo.getPosX(DistanceUnit.MM);
                    margin = target + current;

                    telemetry.addData("Margin", margin);

                    telemetry.update();


                    frontLeftMotor.setPower(power);
                    backLeftMotor.setPower(power);
                    frontRightMotor.setPower(power);
                    backRightMotor.setPower(power);


                }
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);

        }
        private void driveToPosNew(double targetX, double targetY) {
            odo.update();

            while (opModeIsActive()) {
                odo.update();

                double currentX = odo.getPosX(DistanceUnit.MM);
                double currentY = odo.getPosY(DistanceUnit.MM);

                double errorX = targetX - currentX;
                double errorY = targetY - currentY;

                // Stop if close enough
                if (abs(errorX) < 30 && abs(errorY) < 30) break;

                // Scale error to motor power
                double x = 0.001 * errorX;
                double y = -0.001 * errorY;

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
                double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

                // Apply minimum power only if nonzero
                if (abs(rotX) > 0.01 && abs(rotX) < 0.15)
                    rotX = Math.signum(rotX) * 0.15;
                if (abs(rotY) > 0.01 && abs(rotY) < 0.15)
                    rotY = Math.signum(rotY) * 0.15;

                // Normalize
                double denominator = Math.max(abs(rotY) + abs(rotX), 1);

                double frontLeftPower  = (rotY + rotX) / denominator;
                double backLeftPower   = (rotY - rotX) / denominator;
                double frontRightPower = (rotY - rotX) / denominator;
                double backRightPower  = (rotY + rotX) / denominator;

                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);

                telemetry.addData("Error X", errorX);
                telemetry.addData("Error Y", errorY);
                telemetry.addData("rotX", rotX);
                telemetry.addData("rotY", rotY);
                telemetry.update();
            }

            // Stop all motors
            brake();
        }


        private void initAuto() {
            odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
            //        odo.setv ts(101.6, 95.25 ); //these are tuned for 3110-0002-0001 Product Insight #1
            odo.setOffsets(130, 40, DistanceUnit.MM ); //took on 12/20 by Rohan
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
            //odo.recalibrateIMU();
            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Heading Scalar", odo.getYawScalar());
            telemetry.update();

            frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
            backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
            frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
            backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


/*

            // Retrieve the IMU from the hardware map
            imu = hardwareMap.get(IMU.class, "imu");
            //        imu = (IMU) hardwareMap.get(BNO055IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            imu.resetYaw();

            ElapsedTime timer = new ElapsedTime();

*/




        }

        private void brake() {
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }
    }

