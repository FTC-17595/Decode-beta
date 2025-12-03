package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main Mecanum TeleOp Red")
public class MainMecanumTeleOpRed extends LinearOpMode {

    private ArtifactHandlingSystem artifactHandlingSystem;
    private RobotControls robotControls;
    private DriveTrain driveTrain;
    private ColorDetection colorDetection;
    private AprilTagAligner aprilTagAligner;

    @Override
    public void runOpMode() throws InterruptedException {
        artifactHandlingSystem = new ArtifactHandlingSystem(this);
        robotControls = new RobotControls(this);
        driveTrain = new DriveTrain(this);
        colorDetection = new ColorDetection(this);
        aprilTagAligner = new AprilTagAligner(this, driveTrain, 24);

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

            driveTrain.adjustTurnSpeed();
            driveTrain.resetYaw();
            artifactHandlingSystem.shootingSystem(robotControls.shootArtifact, robotControls.motorBrake);
            artifactHandlingSystem.flapSystem(robotControls.flapArtifact);
            artifactHandlingSystem.intakeSystem(robotControls.intakeArtifact, robotControls.rejectIntakeArtifact);
            artifactHandlingSystem.adjustShootingFactor(robotControls.increaseFactor, robotControls.decreaseFactor);
            artifactHandlingSystem.autoShootingSystemTeleOp(robotControls.autoShoot);
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
        driveTrain.displayTelemetry();
        artifactHandlingSystem.displayTelemetry();
        colorDetection.displayTelemetry();
        robotControls.displayTelemetry();
        aprilTagAligner.displayTelemetry();

        telemetry.update();
    }
}
