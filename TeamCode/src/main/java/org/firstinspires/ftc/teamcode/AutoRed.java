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
        return !opModeIsActive() || runtime.seconds() >= 30.0;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        AlignToTag alignToTag = new AlignToTag();
        initAuto();
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && loopFinished && runtime.seconds() < 30) {

            autoMovement.gyroTurnToAngle(-22);
            if (stopIfNeeded()) return;

            alignToTag();
            if (stopIfNeeded()) return;

            autoMovement.shootAutoArtifactFar();
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(22);
            if (stopIfNeeded()) return;

            odo.resetPosAndIMU();
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(857);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-90);
            if (stopIfNeeded()) return;

            autoMovement.intakeRun();
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(1300);
            if (stopIfNeeded()) return;

            sleep(700);
            if (stopIfNeeded()) return;

            autoMovement.intakeSystemAuto(false, false);
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(-1220);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(90);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(-350);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-31);
            if (stopIfNeeded()) return;

            alignToTag();
            if (stopIfNeeded()) return;

            autoMovement.shootAutoArtifactFar();
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(31);
            if (stopIfNeeded()) return;

            sleep(500);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(180);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-90);
            if (stopIfNeeded()) return;

            autoMovement.intakeRun();
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(1000);
            if (stopIfNeeded()) return;

            sleep(700);
            if (stopIfNeeded()) return;

            autoMovement.intakeSystemAuto(false, false);
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(-1000);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(90);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(-400);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-35);
            if (stopIfNeeded()) return;

            alignToTag();
            if (stopIfNeeded()) return;

            autoMovement.shootAutoArtifactFar();
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(35);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(180);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-90);
            if (stopIfNeeded()) return;

            autoMovement.intakeRun();
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(1000);
            if (stopIfNeeded()) return;

            sleep(700);
            if (stopIfNeeded()) return;

            autoMovement.intakeSystemAuto(false, false);
            if (stopIfNeeded()) return;

            autoMovement.PinpointY(-1000);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(90);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(-600);
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-35);
            if (stopIfNeeded()) return;

            alignToTag();
            if (stopIfNeeded()) return;

            autoMovement.shootAutoArtifactFar();
            if (stopIfNeeded()) return;

            autoMovement.gyroTurnToAngle(-55);
            if (stopIfNeeded()) return;

            autoMovement.PinpointX(-600);
            if (stopIfNeeded()) return;

            odo.resetPosAndIMU();
            if (stopIfNeeded()) return;

            loopFinished = false;

            try {
                if (runtime.seconds() >= 30.0) {
                    telemetry.addLine("Gooooood booooooy. Auto complete. :)");
                    telemetry.update();
                    sleep(1000);
                }
                telemetry.update();

                obj.celebrateToggle(true);
                sleep(2500);
                obj.celebrateToggle(false);

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // driveToPos, AlignToTag, getLatestTag stay unchanged

    private void initAuto() {

        autoMovement = new AutoMovement(this);
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
