package org.firstinspires.ftc.teamcode.DisabledClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ArtifactHandlingSystem;

@Autonomous(name = "Auto Outtake Preloads")
@Disabled
public class AutoOuttakePreloads extends LinearOpMode {

    private ArtifactHandlingSystem artifactHandlingSystem;
    private static final long SPINUP_MS = 1500;
    private static final long SERVO_UP_MS = 250;
    private static final long SERVO_RESET_MS = 150;
    private static final long FEED_MS = 1000;
    private static final long FEED_SETTLE_MS = 150;
    private static final long PREFIRE_WAIT_MS = 200;


    @Override
    public void runOpMode() {
        artifactHandlingSystem = new ArtifactHandlingSystem(this);
        artifactHandlingSystem.configureMotorModes();
        artifactHandlingSystem.flapSystem(false);
        artifactHandlingSystem.intakeSystem(false, false, 0);

        telemetry.addLine("Ready to launch preloads");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        launchPreloads();
        telemetry.addLine("Preloads launched");
        telemetry.update();
    }

    private void launchPreloads() {
        artifactHandlingSystem.shootingSystem(1f, 0f);
        sleep(SPINUP_MS);

        if (!prepareArtifact(false)) {
            shutdownSystems();
            return;
        }

        fireArtifact();

        for (int i = 1; i < 4 && opModeIsActive(); i++) {
            if (!prepareArtifact(true)) {
                break;
            }
            fireArtifact();
        }

        shutdownSystems();
    }

    private void fireArtifact() {
        if (!opModeIsActive()) {
            return;
        }
        artifactHandlingSystem.flapSystem(true);
        sleep(SERVO_UP_MS);
        artifactHandlingSystem.flapSystem(false);
        sleep(SERVO_RESET_MS);
    }

    private boolean prepareArtifact(boolean useFeed) {
        if (!opModeIsActive()) {
            return false;
        }

        if (useFeed) {
            artifactHandlingSystem.intakeSystem(true, false, 0);
            sleep(FEED_MS);
            artifactHandlingSystem.intakeSystem(false, false, 0);
        } else {
            sleep(FEED_MS);
        }

        if (!opModeIsActive()) {
            return false;
        }

        sleep(FEED_SETTLE_MS);

        if (!opModeIsActive()) {
            return false;
        }

        sleep(PREFIRE_WAIT_MS);
        return opModeIsActive();
    }

    private void shutdownSystems() {
        artifactHandlingSystem.shootingSystem(0f, 0f);
        artifactHandlingSystem.intakeSystem(false, false, 0);
        artifactHandlingSystem.flapSystem(false);
    }
}