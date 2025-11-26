package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "Autonomous - Blue Alliance")
public class AutoBlue extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    private AutoMovement autoMovement;
    GoBildaPinpointDriver odo;
    DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    RobotAnimations obj;

    int counter = 0;
    boolean PPG = false;
    boolean PGP = false;
    boolean GPP = false;
    boolean loopFinished = true;
    IMU imu;
    AprilTagProcessor tagProcessor;

    private boolean stopIfNeeded() {
        return !opModeIsActive() || getRuntime() >= 30.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        AlignToTag alignToTag = new AlignToTag();
        initAuto();
        waitForStart();
        runtime.reset();

        if (opModeIsActive() && loopFinished && runtime.seconds() < 30) {

            while (!isStopRequested() && loopFinished) {

                alignToTag();
                if (stopIfNeeded()) return;

                autoMovement.shootAutoArtifactFar();
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-22);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(1500);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(90);
                if (stopIfNeeded()) return;

                autoMovement.intakeRun();
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(-1300);
                if (stopIfNeeded()) return;

                sleep(700);
                if (stopIfNeeded()) return;

                autoMovement.intakeSystemAuto(false, false);
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(1220);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-90);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(-900);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(31);
                if (stopIfNeeded()) return;

                alignToTag();
                if (stopIfNeeded()) return;

                autoMovement.shootAutoArtifactFar();
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-31);
                if (stopIfNeeded()) return;

                sleep(500);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(180);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(90);
                if (stopIfNeeded()) return;

                autoMovement.intakeRun();
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(-1000);
                if (stopIfNeeded()) return;

                sleep(700);
                if (stopIfNeeded()) return;

                autoMovement.intakeSystemAuto(false, false);
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(1000);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-90);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(-400);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(35);
                if (stopIfNeeded()) return;

                alignToTag();
                if (stopIfNeeded()) return;

                autoMovement.shootAutoArtifactFar();
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-35);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(180);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(90);
                if (stopIfNeeded()) return;

                autoMovement.intakeRun();
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(-1000);
                if (stopIfNeeded()) return;

                sleep(700);
                if (stopIfNeeded()) return;

                autoMovement.intakeSystemAuto(false, false);
                if (stopIfNeeded()) return;

                autoMovement.PinpointY(1000);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(-90);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(-600);
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(35);
                if (stopIfNeeded()) return;

                alignToTag();
                if (stopIfNeeded()) return;

                autoMovement.shootAutoArtifactFar();
                if (stopIfNeeded()) return;

                autoMovement.gyroTurnToAngle(55);
                if (stopIfNeeded()) return;

                autoMovement.PinpointX(-600);
                if (stopIfNeeded()) return;

                odo.resetPosAndIMU();

                loopFinished = true;
            }
        }
    }

    private void driveToPos(double targetX, double targetY) {
        odo.update();
        boolean telemAdded = true;

        while (opModeIsActive() &&
                (Math.abs(targetX + odo.getPosX(DistanceUnit.MM)) > 30 ||
                        Math.abs(targetY - odo.getPosY(DistanceUnit.MM)) > 30)) {

            odo.update();
            double x = 0.001 * (targetX + odo.getPosX(DistanceUnit.MM));
            double y = -0.001 * (targetY - odo.getPosY(DistanceUnit.MM));

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotY = y * Math.cos(-botHeading) - x * Math.sin(-botHeading);
            double rotX = y * Math.sin(-botHeading) + x * Math.cos(-botHeading);

            if (!telemAdded) {
                telemetry.addData("x: ", x);
                telemetry.addData("y: ", y);
                telemetry.addData("rotX: ", rotX);
                telemetry.addData("rotY: ", rotY);
                telemetry.update();
                telemAdded = true;
            }

            if (Math.abs(rotX) < 0.15) rotX = Math.signum(rotX) * 0.15;
            if (Math.abs(rotY) < 0.15) rotY = Math.signum(rotY) * 0.15;

            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);

            double frontLeftPower = (rotX + rotY) / denominator;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }

        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void AlignToTag(AprilTagDetection tag) {
        double error, drivePower;

        ElapsedTime alignTimer = new ElapsedTime();

        error = tag.ftcPose.yaw;

        while (opModeIsActive() &&
                alignTimer.seconds() < 2.0 &&
                Math.abs(error) > 1.0) {

            odo.update();

            AprilTagDetection currentTag = getLatestTag();
            if (currentTag == null) {
                telemetry.addLine("Tag lost — stopping alignment.");
                break;
            }

            error = currentTag.ftcPose.yaw;
            drivePower = error / 50.0;

            if (drivePower > 0) drivePower = Math.max(drivePower, 0.35);
            else drivePower = Math.min(drivePower, -0.35);

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
            return tagProcessor.getDetections().get(0);
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
            telemetry.addData("Heading IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            driveMotorsPower = error / 50;

            if ((driveMotorsPower < 0.35) && (driveMotorsPower > 0)) driveMotorsPower = 0.35;
            else if ((driveMotorsPower > -0.35) && (driveMotorsPower < 0)) driveMotorsPower = -0.35;

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
        autoMovement = new AutoMovement(this);

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(65, 142, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        this.frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        this.imu.initialize(parameters);
        this.imu.resetYaw();

        ElapsedTime timer = new ElapsedTime();

        if (timer.seconds() >= 1.0) {
            counter++;
            timer.reset();
            telemetry.addData("Counter:", counter);
            telemetry.update();
        }
    }
}
