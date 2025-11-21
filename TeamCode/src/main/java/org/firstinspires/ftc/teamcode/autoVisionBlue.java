package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Autonomous - Vision")
@Disabled
public class autoVisionBlue extends LinearOpMode {

    private DecodeAuto decodeAuto;
    private DriveTrain driveTrain;
    private AprilTagAligner aprilTagAligner;
    GoBildaPinpointDriver odo;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    int counter = 0;
    boolean PPG = false;
    boolean  PGP = false;
    boolean GPP = false;
    boolean loopFinished = true;
    IMU imu;
    AprilTagProcessor tagProcessor;
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();
        
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        
        if (isStopRequested()) {
            return;
        }
        
        // Align to AprilTag with timeout
        ElapsedTime alignmentTimer = new ElapsedTime();
        double alignmentTimeout = 10.0; // 10 second timeout
        
        telemetry.addLine("Starting alignment...");
        telemetry.update();
        
        // Loop until aligned, timeout, or opmode stops
//        while (opModeIsActive() &&
//               !aprilTagAligner.isAligned() &&
//               alignmentTimer.seconds() < alignmentTimeout) {
//
//            // Update detection before aligning
//            aprilTagAligner.updateDetection();
//            // Perform alignment
//            aprilTagAligner.alignToTag();
//
//            // Display telemetry for debugging
//            aprilTagAligner.displayTelemetry();
//            telemetry.addData("Alignment Time", "%.1f / %.1f sec", alignmentTimer.seconds(), alignmentTimeout);
//            telemetry.addData("Alignment Status", aprilTagAligner.getAlignState());
//            telemetry.update();
//
//            // Small delay to prevent overwhelming the loop
//            sleep(20);
//        }
//
//        // Stop alignment when done or timeout
//        aprilTagAligner.stopAlign();
//
//        // Stop all motors
//        driveTrain.driveRobotCentric(0.0, 0.0, 0.0);
//
//        if (aprilTagAligner.isAligned()) {
//            telemetry.addLine("Alignment complete!");
//        } else if (alignmentTimer.seconds() >= alignmentTimeout) {
//            telemetry.addLine("Alignment timeout reached");
//        } else {
//            telemetry.addLine("OpMode stopped");
//        }
//        telemetry.update();
//        sleep(1000);

        // Continue with rest of autonomous code here
//            decodeAuto.gyroTurnToAngle(-22);
//            odo.resetPosAndIMU();
//            decodeAuto.PinpointX(607);
//            decodeAuto.gyroTurnToAngle(90);
//
//            decodeAuto.intakeRun();
////            sleep(1000);
//            decodeAuto.PinpointYBlue(900);
//            sleep(700);
//            decodeAuto.intakeSystemAuto(false,false);
//            decodeAuto.PinpointYBlue(-800);
//            gyroTurnToAngle(-90);
//            decodeAuto.PinpointX(-350);
//            decodeAuto.gyroTurnToAngle(19);
//            decodeAuto.shootAutoArtifactFar();
//            decodeAuto.gyroTurnToAngle(-19);
//            decodeAuto.PinpointX(100);
//            decodeAuto.gyroTurnToAngle(90);




/*  TODO: Add in logic for the vision
        if (PPG == true) {
            driveToPos(-514.710,678.612);
            gyroTurnToAngle(-23);
            driveToPos(-514.710,700.612);
            driveToPos(0,0);
            gyroTurnToAngle(23);
            decodeAuto.shootAutoArtifact();
            // the PPG line
        } else if (PGP == true) {

            driveToPos(-514.710,678.612);
            gyroTurnToAngle(-23);
            driveToPos(-514.710,700.612);
            driveToPos(0,0);
            gyroTurnToAngle(23);
            decodeAuto.shootAutoArtifact();

        } else if (GPP == true) {
            // The GPP line
            driveToPos(12,12);
        }

*/
    }

    // Drives the robot toward a given (X, Y) coordinate using odometry and IMU heading
    private void driveToPos(double targetX, double targetY) {
        // Update odometry before starting
        odo.update();

        boolean telemAdded = true;  // Flag so telemetry is printed only once

        // Keep driving while opmode is active AND the robot is more than 30 cm away in X or Y
        while (opModeIsActive() &&
                (Math.abs(targetX + odo.getPosX(DistanceUnit.MM)) > 30 ||
                        Math.abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

            // Update odometry each loop to get the latest position
            odo.update();

            // Compute distance from target in X and Y, scaled down for motor power
            // The 0.001 factor converts cm error into a smaller motor power signal
            // Negative Y compensates for coordinate orientation differences
            double x = 0.001 * (targetX + odo.getPosX(DistanceUnit.MM));
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
//            telemetry.addData("Y:", odo.getPosY(DistanceUnit.MM));
//            telemetry.addData("X:", odo.getPosX(DistanceUnit.MM));
//            telemetry.update();
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

            telemetry.addData("Y:", odo.getPosY(DistanceUnit.MM));
            telemetry.addData("X:", -odo.getPosX(DistanceUnit.MM));
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
            odo.update();
            telemetry.addData("X: ", -odo.getPosX(DistanceUnit.MM));
            telemetry.addData("Y: ", odo.getPosY(DistanceUnit.MM));
//                telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

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

//    private void initAuto() {
//        decodeAuto = new DecodeAuto(this);
//        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        //        odo.setv ts(101.6, 95.25 ); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setOffsets(65, 142, DistanceUnit.MM ); // Old values: 150, 60
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//        odo.recalibrateIMU();
//
//        this.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
//        this.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
//        this.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
//        this.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
//
//        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//
//
//        // Retrieve the IMU from the hardware map
//        this.imu = hardwareMap.get(IMU.class, "imu");
//        //        imu = (IMU) hardwareMap.get(BNO055IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        this.imu.initialize(parameters);
//        this.imu.resetYaw();
//        // ---------------- APRILTAG SETUP ----------------
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawAxes(true)
//                .build();
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(tagProcessor)
//                .build();
//
//        ElapsedTime timer = new ElapsedTime();
//
//        if (timer.seconds() >= 1.0) {
//            counter++;
//            timer.reset();
//            telemetry.addData("Counter:", counter);
//            telemetry.update();
//        }
//        tagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawAxes(true)
//                .build();
//
//        VisionPortal visionPortal = new VisionPortal.Builder()
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(tagProcessor)
//                .build();
//
//// FIX: pass processor to DecodeAuto instance
//        decodeAuto.setTagProcessor(tagProcessor);
//
//
//
//
//
//    }
private void initAuto() {

    // Initialize DecodeAuto helper
    decodeAuto = new DecodeAuto(this);
    driveTrain = new DriveTrain(this);
    driveTrain.configureMotorModes(); // Configure DriveTrain motors
    aprilTagAligner = new AprilTagAligner(this, driveTrain, 24);

    // -----------------------
    // ODOMETRY / PINPOINT
    // -----------------------
    odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

    odo.setOffsets(65, 142, DistanceUnit.MM);
    odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    odo.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
    );

    odo.resetPosAndIMU();
    odo.recalibrateIMU();


    // -----------------------
    // MOTORS
    // -----------------------
    frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");

    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Reverse the left side
    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    // -----------------------
    // IMU
    // -----------------------
    imu = hardwareMap.get(IMU.class, "imu");

    IMU.Parameters imuParams = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    imu.initialize(imuParams);
    imu.resetYaw();


    telemetry.addLine("Auto Initialized");
    telemetry.update();
}

}

