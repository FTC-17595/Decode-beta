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

@Autonomous(name = "Autonomous - Red Alliance")
public class AutoRed extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    private DecodeAuto decodeAuto;
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

    @Override
    public void runOpMode() throws InterruptedException {
        void callFromAlignToTag();
        AlignToTag alignToTag = new AlignToTag();
        initAuto();
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        if (opModeIsActive() && loopFinished) {
            if (runtime.seconds() >= 30.0) {
                telemetry.addLine("Auto shut down: timer end");
                telemetry.update();
                requestOpModeStop();
            }

            boolean showComplete = false;

            alignToTag();
            decodeAuto.shootAutoArtifactFar();
            decodeAuto.gyroTurnToAngle(22);
            odo.resetPosAndIMU();
            decodeAuto.PinpointX(607);
            decodeAuto.gyroTurnToAngle(-90);
            decodeAuto.intakeRun();
            decodeAuto.PinpointY(1300);
            sleep(700);
            decodeAuto.intakeSystemAuto(false, false);
            decodeAuto.PinpointY(-1220);
            decodeAuto.gyroTurnToAngle(90);
            decodeAuto.PinpointX(-200);
            decodeAuto.gyroTurnToAngle(-31);
            alignToTag();
            decodeAuto.shootAutoArtifactFar();
            decodeAuto.gyroTurnToAngle(31);
            sleep(500);
            odo.resetPosAndIMU();
            decodeAuto.PinpointX(180);
            decodeAuto.gyroTurnToAngle(-90);
            decodeAuto.intakeRun();
            decodeAuto.PinpointY(-1000);
            sleep(700);
            decodeAuto.intakeSystemAuto(false, false);
            decodeAuto.PinpointY(-1100);
            decodeAuto.gyroTurnToAngle(90);
            decodeAuto.PinpointX(-400);
            decodeAuto.gyroTurnToAngle(-35);
            alignToTag();
            decodeAuto.shootAutoArtifactFar();
            decodeAuto.gyroTurnToAngle(35);
            odo.resetPosAndIMU();
            decodeAuto.PinpointX(180);
            decodeAuto.gyroTurnToAngle(-90);
            decodeAuto.intakeRun();
            decodeAuto.PinpointY(-1000);
            sleep(700);
            decodeAuto.intakeSystemAuto(false, false);
            decodeAuto.PinpointY(-1100);
            decodeAuto.gyroTurnToAngle(90);
            decodeAuto.PinpointX(-600);
            decodeAuto.gyroTurnToAngle(-35);
            alignToTag();
            decodeAuto.shootAutoArtifactFar();
            decodeAuto.gyroTurnToAngle(35);
            decodeAuto.PinpointX(-600);

            loopFinished = false;

            try {
                showComplete = true;
                if (showComplete) {
                    telemetry.addLine("Gooooood booooooy. Auto complete.");
                    telemetry.update();
                    sleep(1000);
                    showComplete = false;
                }
                telemetry.update();

                obj.celebrateToggle(true);
                sleep(500);
                obj.celebrateToggle(false);

            } catch (InterruptedException e) {
                e.printStackTrace();
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
        double error = tag.ftcPose.yaw;

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            odo.update();

            AprilTagDetection currentTag = getLatestTag();
            if (currentTag == null) {
                telemetry.addLine("Tag lost — stopping alignment.");
                break;
            }

            error = currentTag.ftcPose.yaw;
            double drivePower = error / 50.0;

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

    private void initAuto() {

        decodeAuto = new DecodeAuto(this);
        obj = new RobotAnimations(this);

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(65, 142, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
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
