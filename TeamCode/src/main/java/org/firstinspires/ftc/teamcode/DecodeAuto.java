package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.Locale;

public class DecodeAuto {


    private final DcMotorEx outtakeMotor;
    private  DcMotor intakeMotor;
    private  DcMotor containerMotor;
    //    private final DcMotor leftContainerMotor;
//    private final DcMotor rightContainerMotor;
    private Servo flapServo;
    private final LinearOpMode linearOpMode;
    private double launchFactor;
    private double launchVelocity;

    GoBildaPinpointDriver odo;
    IMU imu;

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public DecodeAuto(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        // Hardware initialization
        this.frontLeftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.frontRightMotor = linearOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backLeftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.backRightMotor = linearOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");

        this.outtakeMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.get(DcMotor.class, "containerMotor");
        this.flapServo = linearOpMode.hardwareMap.get(Servo.class, "flapServo");
        
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);
//        leftContainerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightContainerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchVelocity = AutoConstants.LONG_RANGE_VELOCITY;


        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
        odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
    }

//    public void OuttakeSystem(float distance) {
//        if (distance > 1) {
//            outtakeMotor.setPower(AutoConstants.LONG_SHOOTING_FACTOR);
//        } else if (distance == 0.5) {
//            outtakeMotor.setPower(AutoConstants.SHORT_SHOOTING_FACTOR);
//        } else {
//            outtakeMotor.setPower(0);
//        }
//    }

    public void OuttakeSystemFar(boolean run) {
        if (run) {
            outtakeMotor.setVelocity(AutoConstants.LONG_RANGE_VELOCITY);
//                        outtakeMotor.setVelocity(1000);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }
    public void OuttakeSystemNear(boolean run) {
        if (run) {
            outtakeMotor.setVelocity(AutoConstants.SHORT_RANGE_VELOCITY);
//                        outtakeMotor.setVelocity(1000);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setVelocity(launchVelocity * shootArtifact);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setVelocity(-launchVelocity * rejectArtifact);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }
    public void shootAutoArtifactDouble(){
//        Thread ArtifactShoot = new Thread(() -> {
//            long endTime = System.currentTimeMillis() + 4000; // 4 seconds from now
//
//            // Start the outtake motor for shooting
//            OuttakeSystem(1);
//
//            while (System.currentTimeMillis() < endTime) {
//
//                containerMotor.setPower(1);
////                containerSystem(true, false);
//
//            }
//
//
//
//            OuttakeSystem(0);
//            System.out.println("Thread finished its 4-second run!");
//        });
//
//        // Start the thread
//        ArtifactShoot.start();whi
        while (opModeIsActive()) {
            OuttakeSystemFar(true);
//            shootingSystem(1,-1);

            sleep(1000);
            AutoflapSystem(true);
            sleep(370);
            AutoflapSystem(false);
            sleep(370);
            intakeSystemAuto(true, false);
            sleep(1000);
            AutoflapSystem(true);
            sleep(370);
            AutoflapSystem(false);
            sleep(370);
//            intakeSystemAuto(true, false);
//            sleep(1500);
//            AutoflapSystem(true);
//            sleep(670);
//            AutoflapSystem(false);

        }

    }
    public void shootAutoArtifactFar(){
//        while (opModeIsActive()) {
            OuttakeSystemFar(true);
            while (( AutoConstants.LONG_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            sleep((long) AutoConstants.FLAP_SLEEP);
            intakeSystemAuto(true, false);
            OuttakeSystemFar(true);
            while (( AutoConstants.LONG_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            OuttakeSystemFar(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            intakeSystemAuto(true, false);
            while (( AutoConstants.LONG_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            sleep((long) AutoConstants.FLAP_SLEEP);
            OuttakeSystemFar(false);
//        }

    }
    public void shootAutoArtifactNear(){
        while (opModeIsActive()) {
            OuttakeSystemNear(true);
            while (( AutoConstants.SHORT_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            sleep((long) AutoConstants.FLAP_SLEEP);
            intakeSystemAuto(true, false);
            OuttakeSystemNear(true);
            while (( AutoConstants.SHORT_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            OuttakeSystemNear(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            intakeSystemAuto(true, false);
            while (( AutoConstants.SHORT_RANGE_VELOCITY - outtakeMotor.getVelocity()) >= 5) {}
            AutoflapSystem(true);
            sleep((long) AutoConstants.FLAP_SLEEP);
            AutoflapSystem(false);
            sleep((long) AutoConstants.FLAP_SLEEP);
        }

    }
    public void shootAutoArtifactSingle(){
            OuttakeSystemFar(true);
             sleep(3000);
            AutoflapSystem(true);
            sleep(870);
//            AutoflapSystem(false);
        flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);


    }
    public void intakeSystemAuto(boolean intakeArtifact, boolean rejectArtifact) {
        if (intakeArtifact) {
            intakeMotor.setPower(1);
            containerMotor.setPower(0.8);
        } else if (rejectArtifact) {
            intakeMotor.setPower(-1);
            containerMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
        }
    }
    public void intakeRun() {

            intakeMotor.setPower(1);
            containerMotor.setPower(0.8);
//            while (intakeMotor.getPower() > 0.2) {}

    }

    public void intakeStop() {

        intakeMotor.setPower(0);
        containerMotor.setPower(0);

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
                (Math.abs(targetX + odo.getPosX(DistanceUnit.MM)) > 30 ||
                        Math.abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

            // Update odometry each loop to get the latest position
            odo.update();

            // Compute distance from target in X and Y, scaled down for motor power
            // The 0.001 factor converts cm error into a smaller motor power signal
            // Negative Y compensates for coordinate orientation differences
            double x = 0.001 * (targetX + odo.getPosX(DistanceUnit.MM));
            double y = 0.001 * (targetY - odo.getPosY(DistanceUnit.MM));

            // Get the robot's heading (rotation angle) from the IMU in radians
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double botHeading = odo.getHeading(AngleUnit.DEGREES);

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
        odo.resetPosAndIMU();
    }

    public void AutoflapSystem(boolean flapUp) {
        if (flapUp) {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_UP);
        } else {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);
        }
    }


    public void autoShootArtifactsFar() {

        AutoflapSystem(true);
    }

    public void PinpointX(double target) {

        odo.setPosX(0, DistanceUnit.MM);

        double margin = target + odo.getPosX(DistanceUnit.MM);


        while (opModeIsActive() && abs(margin) > 100) {

            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            linearOpMode.telemetry.addData("Position", data);
            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());

            double direction = Math.signum(margin);
            double power = AutoConstants.ARTIFACT_PICKUP_SPEED * direction;
            double current = odo.getPosX(DistanceUnit.MM);
            margin = target + current;

            linearOpMode.telemetry.addData("Margin", margin);

            linearOpMode.telemetry.update();


            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);


        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
//        odo.resetPosAndIMU();
    }


    public void PinpointY(double target) {

        odo.setPosY(0,DistanceUnit.MM);

        double margin = target - odo.getPosY(DistanceUnit.MM);


        while (opModeIsActive() && abs(margin) > 100) {

            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            linearOpMode.telemetry.addData("Position", data);
            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());

            double direction = Math.signum(margin);
            double power = AutoConstants.ARTIFACT_PICKUP_SPEED * direction;
            double current = odo.getPosY(DistanceUnit.MM);
            margin = target - current;

            linearOpMode.telemetry.addData("Margin", margin);

            linearOpMode.telemetry.update();


            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);


        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
//        odo.resetPosAndIMU();
    }

    public void PinpointYBlue(double target) {

        odo.setPosY(0,DistanceUnit.MM);

        double margin = target + odo.getPosY(DistanceUnit.MM);


        while (opModeIsActive() && abs(margin) > 100) {

            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            linearOpMode.telemetry.addData("Position", data);
            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());

            double direction = Math.signum(margin);
            double power = AutoConstants.ARTIFACT_PICKUP_SPEED * direction;
            double current = odo.getPosY(DistanceUnit.MM);
            margin = target + current;

            linearOpMode.telemetry.addData("Margin", margin);

            linearOpMode.telemetry.update();


            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);


        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
//        odo.resetPosAndIMU();
    }

//    public void PinpointY(double target, double speed) {
//
////        odo.resetPosAndIMU();
//
//        double margin = target + odo.getPosY(DistanceUnit.MM);
//
//
//        while (opModeIsActive() && abs(margin) > 30) {
//
//            odo.update();
//
//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            linearOpMode.telemetry.addData("Target:", target);
//            linearOpMode.telemetry.addData("Y:", pos.getY(DistanceUnit.MM));
//            linearOpMode.telemetry.addData("Position", data);
//            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());
//            linearOpMode.telemetry.addData("Margin:",margin);
//
//            double direction = Math.signum(margin);
//            double power = 0.5 * direction;
//            double current = pos.getY(DistanceUnit.MM);
//            margin = target + current;
//            double factor = speed/100;
//            linearOpMode.telemetry.addData("Margin", margin);
//
//            linearOpMode.telemetry.update();
//
//
//            frontLeftMotor.setPower(power*factor);
//            backLeftMotor.setPower(power*factor);
//            frontRightMotor.setPower(power*factor);
//            backRightMotor.setPower(power*factor);
////            frontLeftMotor.setPower(power);
////            backLeftMotor.setPower(power);
////            frontRightMotor.setPower(power);
////            backRightMotor.setPower(power);
//
//        }
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//
//        // odo.resetPosAndIMU();
//    }
//    public void gyroTurnToAngle(double turnAngle) {
//        double error, currentHeadingAngle, driveMotorsPower;
//        imu.resetYaw();
//
//        error = turnAngle;
//
//        while (opModeIsActive() && ((error > 2) || (error < -2))) {
//            odo.update();
//            linearOpMode.telemetry.addData("X: ", -odo.getPosX(DistanceUnit.CM));
//            linearOpMode.telemetry.addData("Y: ", odo.getPosY(DistanceUnit.CM));
////                telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
//            linearOpMode.telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            linearOpMode.telemetry.update();
//
//                /*driveMotorsPower = error / 200;
//
//                if ((driveMotorsPower < 0.2) && (driveMotorsPower > 0)) {
//                    driveMotorsPower = 0.2;
//                } else if ((driveMotorsPower > -0.2) && (driveMotorsPower < 0)) {
//                    driveMotorsPower = -0.2;
//                }*/
//            driveMotorsPower = error / 180;
//
//            if ((driveMotorsPower < 1.8) && (driveMotorsPower > 0)) {
//                driveMotorsPower = 0.35;
//            } else if ((driveMotorsPower > -1.8) && (driveMotorsPower < 0)) {
//                driveMotorsPower = -0.35;
//            }
//            // Positive power causes left turn
//            frontLeftMotor.setPower(-driveMotorsPower);
//            backLeftMotor.setPower(-driveMotorsPower);
//            frontRightMotor.setPower(driveMotorsPower);
//            backRightMotor.setPower(driveMotorsPower);
//
//            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            error = turnAngle - currentHeadingAngle;
//        }
//        frontLeftMotor.setPower(0);
//        backLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        backRightMotor.setPower(0);
//
//
//    }
    public void gyroTurnToAngle(double turnAngle) {
        double error, currentHeadingAngle, driveMotorsPower;
        imu.resetYaw();

        error = turnAngle;

        while (opModeIsActive() && ((error > 1) || (error < -1))) {
            odo.update();
            linearOpMode.telemetry.addData("X: ", -odo.getPosX(DistanceUnit.MM));
            linearOpMode.telemetry.addData("Y: ", odo.getPosY(DistanceUnit.MM));
//                telemetry.addData("Heading Odo: ", Math.toDegrees(odo.getHeading()));
            linearOpMode.telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            linearOpMode.telemetry.update();

            driveMotorsPower = error / 180;

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
            frontLeftMotor.setPower(-driveMotorsPower/2);
            backLeftMotor.setPower(-driveMotorsPower/2);
            frontRightMotor.setPower(driveMotorsPower/2);
            backRightMotor.setPower(driveMotorsPower/2);

            currentHeadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);


    }


}