package org.firstinspires.ftc.teamcode.DisabledClasses;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.DecodeAuto;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Gryo")
@Disabled
public class GyroTest extends LinearOpMode {

    private DecodeAuto decodeAuto;
    private AprilTagDetection aprilTagDetection;
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
        waitForStart();
        if(isStopRequested()) return;

        decodeAuto.gyroTurnToAngle(90);

//        AlignToTag(aprilTagDetection,20);

    }



    private void AlignToTag(AprilTagDetection tag, float tagID) {
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

        private void initAuto() {
        decodeAuto = new DecodeAuto(this);
        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        //        odo.setv ts(101.6, 95.25 ); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setOffsets(150, 60, DistanceUnit.MM ); //took on 12/20 by Rohan
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
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



        // Retrieve the IMU from the hardware map
        this.imu = hardwareMap.get(IMU.class, "imu");
        //        imu = (IMU) hardwareMap.get(BNO055IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
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

