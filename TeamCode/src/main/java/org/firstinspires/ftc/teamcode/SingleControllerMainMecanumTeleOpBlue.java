package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Single Controller Main Mecanum TeleOp Blue")
public class SingleControllerMainMecanumTeleOpBlue extends LinearOpMode {

    private ArtifactHandlingSystem artifactHandlingSystem;
    private SingleControllerControls robotControls;
    private DriveTrain driveTrain;
    private ColorDetection colorDetection;
    private AprilTagAligner aprilTagAligner;

    @Override
    public void runOpMode() throws InterruptedException {
        artifactHandlingSystem = new ArtifactHandlingSystem(this);
        robotControls = new SingleControllerControls(this);
        driveTrain = new DriveTrain(this);
        colorDetection = new ColorDetection(this);
        aprilTagAligner = new AprilTagAligner(this, driveTrain, 20);

        configureMotorModes();

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }
            mainTeleOpLoop();
        } finally {
            if (aprilTagAligner != null) {
                aprilTagAligner.close();
            }
        }
    }

    private void mainTeleOpLoop() throws InterruptedException {
        while (opModeIsActive()) {
            robotControls.updateControls();
            aprilTagAligner.updateDetection();
            artifactHandlingSystem.updateLaunchVelocityForRange(aprilTagAligner.getLastRangeInches(), aprilTagAligner.getTargetedTagId());
            aprilTagAligner.setRangeHoldEnabled(artifactHandlingSystem.getLaunchVelocity() == TeleOpConstants.EXTRA_LONG_RANGE_VELOCITY);

            driveTrain.adjustTurnSpeed();
            driveTrain.resetYaw();
            artifactHandlingSystem.shootingSystem(robotControls.shootArtifact, robotControls.motorBrake);
            artifactHandlingSystem.flapSystem(robotControls.flapArtifact);
            artifactHandlingSystem.manageIntakeWithAutoFeed(
                robotControls.intakeArtifact,
                robotControls.rejectIntakeArtifact,
                robotControls.shootArtifact > 0.1f,
                colorDetection.isArtifactAtBack(),
                colorDetection.getArtifactCount()
            );
            artifactHandlingSystem.adjustShootingFactor(robotControls.increaseFactor, robotControls.decreaseFactor);
            artifactHandlingSystem.switchShootingFactor(robotControls.switchLaunchPower);
            artifactHandlingSystem.checkMotorHealth();
            colorDetection.celebrateToggle(robotControls.celebrate);
            colorDetection.setRGBIndicator();
            colorDetection.setOuttakeIndicatorWithVelocity(
                    artifactHandlingSystem.getLaunchVelocity(),
                    artifactHandlingSystem.getActualVelocity()
            );
            aprilTagAligner.align(robotControls.alignRobot);

            displayTelemetry();
        }
    }

    private void configureMotorModes() {
        artifactHandlingSystem.configureMotorModes();
        driveTrain.configureMotorModes();
        driveTrain.resetYaw();
    }

    private void displayTelemetry() {
        if (robotControls.DEBUG) {
            driveTrain.displayTelemetry();
            artifactHandlingSystem.displayTelemetry();
            colorDetection.displayTelemetry();
            robotControls.displayTelemetry();
            aprilTagAligner.displayTelemetry();

            telemetry.update();
        }
    }
}
