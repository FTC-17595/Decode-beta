package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class AprilTagAligner {

    public enum AlignState {
        IDLE,
        NO_TARGET,
        ALIGNING,
        ALIGNED
    }

    private final LinearOpMode linearOpMode;
    private final DriveTrain driveTrain;
    private final int desiredTagId;

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final FtcDashboard ftcDashboard;

    private final SimplePIDController strafeController = new SimplePIDController(
            TeleOpConstants.ALIGN_STRAFE_KP,
            TeleOpConstants.ALIGN_STRAFE_KI,
            TeleOpConstants.ALIGN_STRAFE_KD);
    private final SimplePIDController turnController = new SimplePIDController(
            TeleOpConstants.ALIGN_TURN_KP,
            TeleOpConstants.ALIGN_TURN_KI,
            TeleOpConstants.ALIGN_TURN_KD);

    private final ElapsedTime loopTimer = new ElapsedTime();

    private AprilTagDetection currentDetection;
    private AprilTagDetection lastDetection;
    private AlignState lastState = AlignState.IDLE;

    private double lastStrafeCommand = 0.0;
    private double lastTurnCommand = 0.0;
    private double lastStrafeError = 0.0;
    private double lastTurnError = 0.0;

    private boolean controllersPrimed = false;

    public AprilTagAligner(LinearOpMode linearOpMode, DriveTrain driveTrain, int desiredTagId) {
        this(linearOpMode, driveTrain, desiredTagId, "Webcam");
    }

    public AprilTagAligner(LinearOpMode linearOpMode, DriveTrain driveTrain, int desiredTagId, String webcamName) {
        this.linearOpMode = linearOpMode;
        this.driveTrain = driveTrain;
        this.desiredTagId = desiredTagId;
        this.ftcDashboard = FtcDashboard.getInstance();

        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(
                        (int) TeleOpConstants.OPENCV_IMAGE_WIDTH,
                        (int) TeleOpConstants.OPENCV_IMAGE_HEIGHT))
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, webcamName))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        loopTimer.reset();
    }

    public void updateDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        currentDetection = null;
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.id == desiredTagId) {
                currentDetection = detection;
                lastDetection = detection;
                break;
            }
        }
        if (currentDetection == null) {

            lastStrafeError = 0.0;
            lastTurnError = 0.0;
        } else {
            lastStrafeError = currentDetection.ftcPose.x;
            lastTurnError = currentDetection.ftcPose.bearing;
        }
    }

    public void alignToTag() {
        updateCoefficients();
        if (!controllersPrimed) {
            controllersPrimed = true;
            strafeController.reset();
            turnController.reset();
            loopTimer.reset();
        }

        if (currentDetection == null) {
            lastState = AlignState.NO_TARGET;
            lastStrafeCommand = 0.0;
            lastTurnCommand = 0.0;
            driveTrain.driveRobotCentric(0.0, 0.0, 0.0);
            return;
        }

        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0.0) {
            dt = 1e-3;
        }

        double rawStrafe = currentDetection.ftcPose.x;
        double rawForward = currentDetection.ftcPose.y;

        double adjustedStrafe = rawStrafe + TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME;
        double adjustedForward = rawForward + TeleOpConstants.CAMERA_OFFSET_Y_IN;

        double turnError = Math.toDegrees(Math.atan2(adjustedStrafe, adjustedForward));

        lastStrafeError = adjustedStrafe;
        lastTurnError = turnError;

        double strafeCommand = strafeController.calculate(adjustedStrafe, dt);
        double turnCommand = turnController.calculate(turnError, dt);

        double appliedStrafeCommand = Range.clip(
                -strafeCommand,
                -TeleOpConstants.ALIGN_MAX_STRAFE_POWER,
                TeleOpConstants.ALIGN_MAX_STRAFE_POWER);
        double appliedTurnCommand = Range.clip(
                turnCommand,
                -TeleOpConstants.ALIGN_MAX_TURN_POWER,
                TeleOpConstants.ALIGN_MAX_TURN_POWER);

        lastStrafeCommand = appliedStrafeCommand;
        lastTurnCommand = appliedTurnCommand;

        driveTrain.driveRobotCentric(0.0, appliedStrafeCommand, appliedTurnCommand);

        boolean strafeAligned = Math.abs(adjustedStrafe) <= TeleOpConstants.ALIGN_STRAFE_TOLERANCE_IN;
        boolean turnAligned = Math.abs(turnError) <= TeleOpConstants.ALIGN_BEARING_TOLERANCE_DEG;

        lastState = (strafeAligned && turnAligned) ? AlignState.ALIGNED : AlignState.ALIGNING;
        lastDetection = currentDetection;
    }

    public void stopAlign() {
        if (controllersPrimed) {
            controllersPrimed = false;
            strafeController.reset();
            turnController.reset();
            loopTimer.reset();
        }
        lastState = AlignState.IDLE;
        lastStrafeCommand = 0.0;
        lastTurnCommand = 0.0;
    }

    public void align(boolean alignRequested) {
        if (alignRequested) {
            alignToTag();
        } else {
            stopAlign();
            driveTrain.setMotorPowers();
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("AprilTag Align State", lastState);
        if (lastDetection != null) {
            linearOpMode.telemetry.addData("AprilTag ID", lastDetection.id);
            linearOpMode.telemetry.addData("AprilTag Range (in)", "%.1f", lastDetection.ftcPose.range);
            linearOpMode.telemetry.addData("AprilTag X/Y (in)", "%.1f / %.1f", lastDetection.ftcPose.x, lastDetection.ftcPose.y);
            linearOpMode.telemetry.addData("AprilTag Bearing/Yaw (deg)", "%.1f / %.1f",
                    lastDetection.ftcPose.bearing, lastDetection.ftcPose.yaw);
        } else {
            linearOpMode.telemetry.addData("AprilTag ID", "none");
        }

        linearOpMode.telemetry.addData("Align Error", "strafe=%.2f in, turn=%.2f deg", lastStrafeError, lastTurnError);
        linearOpMode.telemetry.addData("Align Cmd", "strafe=%.2f, turn=%.2f", lastStrafeCommand, lastTurnCommand);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("state", lastState.name());
        packet.put("strafeErrorIn", lastStrafeError);
        packet.put("turnErrorDeg", lastTurnError);
        packet.put("strafeCommand", lastStrafeCommand);
        packet.put("turnCommand", lastTurnCommand);
        if (lastDetection != null) {
            packet.put("tagId", lastDetection.id);
            packet.put("tagRangeIn", lastDetection.ftcPose.range);
            packet.put("tagXIn", lastDetection.ftcPose.x);
            packet.put("tagBearingDeg", lastDetection.ftcPose.bearing);
        }
        ftcDashboard.sendTelemetryPacket(packet);
    }

    public void close() {
        visionPortal.close();
    }

    private void updateCoefficients() {
        strafeController.setCoefficients(
                TeleOpConstants.ALIGN_STRAFE_KP,
                TeleOpConstants.ALIGN_STRAFE_KI,
                TeleOpConstants.ALIGN_STRAFE_KD);
        turnController.setCoefficients(
                TeleOpConstants.ALIGN_TURN_KP,
                TeleOpConstants.ALIGN_TURN_KI,
                TeleOpConstants.ALIGN_TURN_KD);
    }

    public double getLastRangeInches() {
        return lastDetection != null ? lastDetection.ftcPose.range : Double.NaN;
    }

    public double getTargetedTagId() {
        return lastDetection != null ? lastDetection.id : Double.NaN;
    }
}


