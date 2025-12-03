package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Single Controller TeleOp")
public class SingleControllerTeleOp extends LinearOpMode {

    private ArtifactHandlingSystem artifactHandlingSystem;
    private singleController robotControls;
    private DriveTrain driveTrain;
    private ColorDetection colorDetection;



    @Override
    public void runOpMode() throws InterruptedException {
        artifactHandlingSystem = new ArtifactHandlingSystem(this);
        robotControls = new singleController(this);
        driveTrain = new DriveTrain(this);
        colorDetection = new ColorDetection(this);


        configureMotorModes();

        waitForStart();
        if (isStopRequested()) return;
        mainTeleOpLoop();
    }

    private void mainTeleOpLoop() throws InterruptedException {
        while (opModeIsActive()) {
            robotControls.updateControls();


            driveTrain.adjustTurnSpeed();
            driveTrain.setMotorPowers();
            driveTrain.resetYaw();
            artifactHandlingSystem.shootingSystem(robotControls.shootArtifact, robotControls.motorBrake);
            artifactHandlingSystem.flapSystem(robotControls.flapArtifact);
            artifactHandlingSystem.intakeSystem(robotControls.intakeArtifact, robotControls.rejectIntakeArtifact);
            artifactHandlingSystem.adjustShootingFactor(robotControls.increaseFactor, robotControls.decreaseFactor);
            artifactHandlingSystem.switchShootingFactor(robotControls.switchLaunchPower);
            artifactHandlingSystem.checkMotorHealth();
            colorDetection.celebrateToggle(robotControls.celebrate);
            colorDetection.setRGBIndicator();
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

        telemetry.update();
    }
}
