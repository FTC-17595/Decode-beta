package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.teamcode.AutoConstants.CLASSIFIER_X;
import static org.firstinspires.ftc.teamcode.AutoConstants.CLASSIFIER_Y;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Auto LM1")
public class DecodeLM1Auto extends LinearOpMode {

    boolean debug = true;

    GoBildaPinpointDriver odo;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    int counter = 0;
    boolean PPG = false;
    boolean  PGP = false;
    boolean GPP = false;
    IMU imu;
    AprilTagProcessor tagProcessor;
    @Override
    public void runOpMode() {

        initAuto();
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        AprilTagDetection tag = tagProcessor.getDetections().get(0);




        if (tag.id == 21) {
            GPP = true;

        } else if (tag.id == 22) {
            PGP = true;

        } else if (tag.id == 23){
            PPG = true;
        }

        driveToPos(CLASSIFIER_X, CLASSIFIER_Y);
            gyroTurnToAngle(110);

            ArtifactHandlingSystem artifactSystem = new ArtifactHandlingSystem(linearOpMode);
            artifactSystem.shootAutoArtifact();






/*      TODO: ADD IN THE VALUES FOR  OBELISK
        if (PPG == true) {
            driveToPos(12,12);
            // the PPG line
        } else if (PGP == true) {

            // the PGP line
            driveToPos(12,12);
        } else if (GPP == true) {
            // The GPP line
            driveToPos(12,12);
        }

*/
    }

    private void debug() {
        odo.update();
        telemetry.addData("X: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        if (debug) {

        }
        telemetry.update();
    }

    // Drives the robot toward a given (X, Y) coordinate using odometry and IMU heading
    private void driveToPos(double targetX, double targetY) {
        // Update odometry before starting
        odo.update();

        boolean telemAdded = false;  // Flag so telemetry is printed only once

        // Keep driving while opmode is active AND the robot is more than 30 cm away in X or Y
        while (opModeIsActive() &&
                (Math.abs(targetX - odo.getPosX(DistanceUnit.MM)) > 30 ||
                        Math.abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

            // Update odometry each loop to get the latest position
            debug();

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
            telemetry.update();
        }

        // Stop all motors when target position is reached or opmode ends
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }


    private void AlignToTag(AprilTagDetection tag) {
        double error, drivePower;


        error = tag.ftcPose.yaw;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            odo.update();


            AprilTagDetection currentTag = getLatestTag();
            if (currentTag == null) {
                telemetry.addLine("Tag lost — stopping alignment.");
                break;
            }

            error = currentTag.ftcPose.yaw;


            drivePower = error / 50.0;


            if (drivePower > 0) {
                drivePower = Math.max(drivePower, 0.35);
            } else if (drivePower < 0) {
                drivePower = Math.min(drivePower, -0.35);
            }


            frontLeftMotor.setPower(-drivePower);
            backLeftMotor.setPower(-drivePower);
            frontRightMotor.setPower(drivePower);
            backRightMotor.setPower(drivePower);

            telemetry.addData("Tag Yaw", error);
            telemetry.addData("Drive Power", drivePower);
            telemetry.update();
        }


        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private AprilTagDetection getLatestTag() {
        if (tagProcessor.getDetections().size() > 0) {
            AprilTagDetection aprilTagDetection = tagProcessor.getDetections().get(0);
            return aprilTagDetection;
        }
        return null;
    }


    private void gyroTurnToAngle(double turnAngle) {
            double error, currentHeadingAngle, driveMotorsPower;
            imu.resetYaw();

            error = turnAngle;

            while (opModeIsActive() && ((error > 1) || (error < -1))) {
                driveMotorsPower = error / 200;

                if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
                    driveMotorsPower = 0.2;
                } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
                    driveMotorsPower = -0.2;
                }

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

        private void initAuto() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //        odo.setv ts(101.6, 95.25 ); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(150, 60, DistanceUnit.MM ); //took on 12/20 by Rohan
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

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



        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //        imu = (IMU) hardwareMap.get(BNO055IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        ElapsedTime timer = new ElapsedTime();

        if (timer.seconds() >= 1.0) {
            counter++;
            timer.reset();
            telemetry.addData("Counter:", counter);
            telemetry.update();
        }


    }
}

