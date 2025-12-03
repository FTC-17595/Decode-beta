package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOpConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Locale;

public class AutoMovement {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
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
    AprilTagProcessor tagProcessor;

    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public AutoMovement(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        // Hardware initialization
        this.frontLeftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.frontRightMotor = linearOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backLeftMotor = linearOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.backRightMotor = linearOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");
//        tagProcessor = new AprilTagProcessor();
        this.outtakeMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.get(DcMotor.class, "containerMotor");
        this.flapServo = linearOpMode.hardwareMap.get(Servo.class, "flapServo");

        MotorConfigurationType config = outtakeMotor.getMotorType().clone();
        config.setAchieveableMaxRPMFraction(1.0);
        outtakeMotor.setMotorType(config);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                AutoConstants.kP,
                AutoConstants.kI,
                AutoConstants.kD,
                AutoConstants.kF
        );

        outtakeMotor.setVelocityPIDFCoefficients(AutoConstants.kP, AutoConstants.kI, AutoConstants.kD, AutoConstants.kF);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
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
//

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
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
            if (!linearOpMode.opModeIsActive() || linearOpMode.isStopRequested()) {
                break;
            }

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
            flapServo.setPosition(AutoConstants.FLAP_SERVO_UP);
        } else {
            flapServo.setPosition(AutoConstants.FLAP_SERVO_DOWN);
        }
    }


    public void autoShootArtifactsFar() {

        AutoflapSystem(true);

    }

    public void PinpointX(double target) {

//        odo.setPosX(0, DistanceUnit.MM);
//        sleep(700);

        double margin = target + odo.getPosX(DistanceUnit.MM);


        while (opModeIsActive() && abs(margin) > 30) {
            if (!linearOpMode.opModeIsActive() || linearOpMode.isStopRequested()) {
                break;
            }

            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            linearOpMode.telemetry.addData("Position", data);
            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());
            linearOpMode.telemetry.addData("Margin", margin);

            double direction = Math.signum(margin);
            double power = direction * 0.8;
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



    public void PinpointY(double target, double speed) {

        odo.setPosY(0,DistanceUnit.MM);

        double margin = target - odo.getPosY(DistanceUnit.MM);


        while (opModeIsActive() && abs(margin) > 30) {
            if (!linearOpMode.opModeIsActive() || linearOpMode.isStopRequested()) {
                break;
            }

            odo.update();

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));

            linearOpMode.telemetry.addData("Position", data);
            linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());
            double factor = speed/100;
            double direction = Math.signum(margin);
            double power = (AutoConstants.ARTIFACT_PICKUP_SPEED * direction)* factor;
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
            if (!linearOpMode.opModeIsActive() || linearOpMode.isStopRequested()) {
                break;
            }

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
            if (!linearOpMode.opModeIsActive() || linearOpMode.isStopRequested()) {
                break;
            }

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