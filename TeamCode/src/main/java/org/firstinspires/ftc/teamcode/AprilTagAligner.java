package org.firstinspires.ftc.teamcode;

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
    private final SimplePIDController forwardController = new SimplePIDController(
            TeleOpConstants.ALIGN_FORWARD_KP,
            TeleOpConstants.ALIGN_FORWARD_KI,
            TeleOpConstants.ALIGN_FORWARD_KD);

    private final ElapsedTime loopTimer = new ElapsedTime();

    private AprilTagDetection currentDetection;
    private AprilTagDetection lastDetection;
    private AlignState lastState = AlignState.IDLE;

    private double lastStrafeCommand = 0.0;
    private double lastTurnCommand = 0.0;
    private double lastForwardCommand = 0.0;
    private double lastStrafeError = 0.0;
    private double lastTurnError = 0.0;
    private double lastRangeError = 0.0;

    private boolean controllersPrimed = false;
    private boolean rangeHoldEnabled = false;

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
            forwardController.reset();
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
        double currentRangeIn = currentDetection.ftcPose.range;

        double adjustedStrafe = rawStrafe + TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME;
        double adjustedForward = rawForward + TeleOpConstants.CAMERA_OFFSET_Y_IN;

        double turnError = Math.toDegrees(Math.atan2(adjustedStrafe, adjustedForward));
        double rangeError = currentRangeIn - TeleOpConstants.ALIGN_TARGET_RANGE_EXTRA_LONG_IN;

        lastStrafeError = adjustedStrafe;
        lastTurnError = turnError;
        lastRangeError = rangeError;

        double strafeCommand = strafeController.calculate(adjustedStrafe, dt);
        double turnCommand = turnController.calculate(turnError, dt);
        double forwardCommand = rangeHoldEnabled ? forwardController.calculate(rangeError, dt) : 0.0;

        double appliedStrafeCommand = Range.clip(
                -strafeCommand,
                -TeleOpConstants.ALIGN_MAX_STRAFE_POWER,
                TeleOpConstants.ALIGN_MAX_STRAFE_POWER);
        double appliedTurnCommand = Range.clip(
                turnCommand,
                -TeleOpConstants.ALIGN_MAX_TURN_POWER,
                TeleOpConstants.ALIGN_MAX_TURN_POWER);
        double appliedForwardCommand = rangeHoldEnabled
                ? Range.clip(forwardCommand,
                -TeleOpConstants.ALIGN_MAX_FORWARD_POWER,
                TeleOpConstants.ALIGN_MAX_FORWARD_POWER)
                : 0.0;

        lastStrafeCommand = appliedStrafeCommand;
        lastTurnCommand = appliedTurnCommand;
        lastForwardCommand = appliedForwardCommand;

        driveTrain.driveRobotCentric(appliedForwardCommand, appliedStrafeCommand, appliedTurnCommand);

        boolean strafeAligned = Math.abs(adjustedStrafe) <= TeleOpConstants.ALIGN_STRAFE_TOLERANCE_IN;
        boolean turnAligned = Math.abs(turnError) <= TeleOpConstants.ALIGN_BEARING_TOLERANCE_DEG;
        boolean rangeAligned = !rangeHoldEnabled || Math.abs(rangeError) <= TeleOpConstants.ALIGN_RANGE_TOLERANCE_IN;

        lastState = (strafeAligned && turnAligned && rangeAligned) ? AlignState.ALIGNED : AlignState.ALIGNING;
        lastDetection = currentDetection;
    }

    public void stopAlign() {
        if (controllersPrimed) {
            controllersPrimed = false;
            strafeController.reset();
            turnController.reset();
            forwardController.reset();
            loopTimer.reset();
        }
        lastState = AlignState.IDLE;
        lastStrafeCommand = 0.0;
        lastTurnCommand = 0.0;
        lastForwardCommand = 0.0;
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
        linearOpMode.telemetry.addData("Range Hold (50in)", rangeHoldEnabled);
        if (lastDetection != null) {
            linearOpMode.telemetry.addData("AprilTag ID", lastDetection.id);
            linearOpMode.telemetry.addData("AprilTag Range (in)", "%.1f", lastDetection.ftcPose.range);
            linearOpMode.telemetry.addData("AprilTag X/Y (in)", "%.1f / %.1f", lastDetection.ftcPose.x, lastDetection.ftcPose.y);
            linearOpMode.telemetry.addData("AprilTag Bearing/Yaw (deg)", "%.1f / %.1f",
                    lastDetection.ftcPose.bearing, lastDetection.ftcPose.yaw);
        } else {
            linearOpMode.telemetry.addData("AprilTag ID", "none");
        }

        linearOpMode.telemetry.addData("Align Error", "range=%.2f in, strafe=%.2f in, turn=%.2f deg", lastRangeError, lastStrafeError, lastTurnError);
        linearOpMode.telemetry.addData("Align Cmd", "forward=%.2f, strafe=%.2f, turn=%.2f", lastForwardCommand, lastStrafeCommand, lastTurnCommand);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("state", lastState.name());
        packet.put("rangeErrorIn", lastRangeError);
        packet.put("strafeErrorIn", lastStrafeError);
        packet.put("turnErrorDeg", lastTurnError);
        packet.put("forwardCommand", lastForwardCommand);
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
        forwardController.setCoefficients(
                TeleOpConstants.ALIGN_FORWARD_KP,
                TeleOpConstants.ALIGN_FORWARD_KI,
                TeleOpConstants.ALIGN_FORWARD_KD);
    }

    public double getLastRangeInches() {
        return lastDetection != null ? lastDetection.ftcPose.range : Double.NaN;
    }

    public double getTargetedTagId() {
        return lastDetection != null ? lastDetection.id : Double.NaN;
    }

    public void setRangeHoldEnabled(boolean enabled) {
        this.rangeHoldEnabled = enabled;
        if (!enabled) {
            forwardController.reset();
        }
    }
}


