package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactHandlingSystem {

    private final DcMotorEx outtakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor containerMotor;
    private final Servo flapServo;
    private final LinearOpMode linearOpMode;
    private double launchVelocity;
    private ShootingState shootingState = ShootingState.IDLE;
    private long stateStartTime = 0;
    private int artifactsFired = 0;
    private boolean useFeedForCurrent = false;
    private boolean lastSwitchState = false;

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.dcMotor.get("containerMotor");
        this.flapServo = linearOpMode.hardwareMap.servo.get("flapServo");
        this.linearOpMode = linearOpMode;
    }

    public void configureMotorModes() {
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);

        launchVelocity = TeleOpConstants.SHORT_RANGE_VELOCITY;
    }

    public void intakeSystem(boolean intakeArtifact, boolean rejectArtifact) {
        if (intakeArtifact) {
            intakeMotor.setPower(1);
            containerMotor.setPower(1);
        } else if (rejectArtifact) {
            intakeMotor.setPower(-1);
            containerMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
        }
    }

    public void flapSystem(boolean flapUp) {
        if (flapUp) {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_UP);
        } else {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);
        }
    }

    public void shootAutoArtifact(){
        Thread ArtifactShoot = new Thread(() -> {
            long endTime = System.currentTimeMillis() + 4000; // 4 seconds from now

            // Start the outtake motor at target velocity
            shootingSystem(1, 0);

            while (System.currentTimeMillis() < endTime) {
                containerMotor.setPower(1);
            }

            shootingSystem(0, 0);
            System.out.println("Thread finished its 4-second run!");
        });

        ArtifactShoot.start();
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0.1) {
            outtakeMotor.setVelocity(launchVelocity);
        } else if (rejectArtifact > 0.1) {
            outtakeMotor.setVelocity(-launchVelocity);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public boolean autoShootingSystemTeleOp(boolean startShootingSequence) {
        // Initialize the sequence when trigger is pressed
        if (startShootingSequence && shootingState == ShootingState.IDLE) {
            shootingState = ShootingState.SPINUP;
            stateStartTime = System.currentTimeMillis();
            artifactsFired = 0;
            linearOpMode.telemetry.addLine("Starting auto-shoot sequence...");
            return true;
        }

        // State machine logic
        long currentTime = System.currentTimeMillis();
        long elapsed = currentTime - stateStartTime;

        switch (shootingState) {
            case IDLE:
                // Do nothing, waiting for trigger
                return false;

            case SPINUP:
                // Start the shooter motor and wait for it to reach target velocity
                shootingSystem(1f, 0f);
                linearOpMode.telemetry.addData("State", "Spinning up shooter...");
                linearOpMode.telemetry.addData("Time remaining", "%.1f s", (TeleOpConstants.SPINUP_MS - elapsed) / 1000.0);

                if (elapsed >= TeleOpConstants.SPINUP_MS) {
                    shootingState = ShootingState.PREPARE_FIRST;
                    stateStartTime = currentTime;
                    useFeedForCurrent = false; // First artifact doesn't use feed
                }
                return true;

            case PREPARE_FIRST:
            case PREPARE_NEXT:
                // Prepare the artifact for firing
                boolean isFirst = (shootingState == ShootingState.PREPARE_FIRST);

                if (useFeedForCurrent && elapsed < TeleOpConstants.FEED_MS) {
                    // Run intake/container to feed next artifact
                    intakeSystem(true, false);
                    linearOpMode.telemetry.addData("State", "Feeding artifact %d...", artifactsFired + 1);
                } else if (elapsed < TeleOpConstants.FEED_MS) {
                    // For first artifact, just wait (no feed needed)
                    intakeSystem(false, false);
                    linearOpMode.telemetry.addData("State", "Preparing artifact %d...", artifactsFired + 1);
                } else if (elapsed < TeleOpConstants.FEED_MS + TeleOpConstants.FEED_SETTLE_MS) {
                    // Stop intake and let artifact settle
                    intakeSystem(false, false);
                    linearOpMode.telemetry.addData("State", "Settling artifact %d...", artifactsFired + 1);
                } else if (elapsed >= TeleOpConstants.FEED_MS + TeleOpConstants.FEED_SETTLE_MS + TeleOpConstants.PREFIRE_WAIT_MS) {
                    // Preparation complete, move to fire state
                    shootingState = isFirst ? ShootingState.FIRE_FIRST : ShootingState.FIRE_NEXT;
                    stateStartTime = currentTime;
                }
                return true;

            case FIRE_FIRST:
            case FIRE_NEXT:
                // Execute the firing sequence (raise flap, wait, lower flap)
                if (elapsed < TeleOpConstants.SERVO_UP_MS) {
                    // Raise the flap to release artifact
                    flapSystem(true);
                    linearOpMode.telemetry.addData("State", "Firing artifact %d - raising flap", artifactsFired + 1);
                } else if (elapsed < TeleOpConstants.SERVO_UP_MS + TeleOpConstants.SERVO_RESET_MS) {
                    // Lower the flap back down
                    flapSystem(false);
                    linearOpMode.telemetry.addData("State", "Firing artifact %d - resetting flap", artifactsFired + 1);
                } else {
                    // Firing complete
                    artifactsFired++;
                    linearOpMode.telemetry.addData("Artifacts Fired", artifactsFired);

                    // Check if we should continue or complete
                    if (artifactsFired >= TeleOpConstants.MAX_ARTIFACTS) {
                        shootingState = ShootingState.COOLDOWN;
                    } else {
                        shootingState = ShootingState.PREPARE_NEXT;
                        useFeedForCurrent = true; // All subsequent artifacts use feed
                    }
                    stateStartTime = currentTime;
                }
                return true;

            case COOLDOWN:
                // Brief cooldown before completing
                if (elapsed >= 100) {
                    shootingState = ShootingState.COMPLETE;
                    stateStartTime = currentTime;
                }
                return true;

            case COMPLETE:
                // Shutdown all systems and return to idle
                shootingSystem(0f, 0f);
                intakeSystem(false, false);
                flapSystem(false);
                linearOpMode.telemetry.addData("Auto-shoot", "Complete! %d artifacts fired", artifactsFired);

                // Stay in complete state briefly so user can see the message
                if (elapsed >= 1000) {
                    shootingState = ShootingState.IDLE;
                }
                return false;

            default:
                shootingState = ShootingState.IDLE;
                return false;
        }
    }

    public void adjustShootingFactor(boolean increase, boolean decrease) {
        if (increase) {
            launchVelocity += TeleOpConstants.VELOCITY_INCREMENT;
        } else if (decrease) {
            launchVelocity -= TeleOpConstants.VELOCITY_INCREMENT;
        }
        // Clamp velocity to safe operating limits
        // Max theoretical = 11,200 ticks/sec;
        launchVelocity = Math.max(0, Math.min(launchVelocity, TeleOpConstants.MAX_VELOCITY));
    }

    public void switchShootingFactor(boolean switch_f) {
        // Only toggle when switch_f transitions from false to true (rising edge)
        if (switch_f && !lastSwitchState) {
            launchVelocity = (launchVelocity == TeleOpConstants.SHORT_RANGE_VELOCITY)
                    ? TeleOpConstants.LONG_RANGE_VELOCITY
                    : TeleOpConstants.SHORT_RANGE_VELOCITY;
        }
        lastSwitchState = switch_f;
    }

    public void shootingSystemAuto(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setVelocity(AutoConstants.LONG_RANGE_VELOCITY);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setVelocity(-launchVelocity * rejectArtifact);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public double getLaunchVelocity() {
        return launchVelocity;
    }

    public double getActualVelocity() {
        return outtakeMotor.getVelocity();
    }

    public void checkMotorHealth() {
        if (outtakeMotor.isOverCurrent()) {
            linearOpMode.telemetry.addData("WARNING", "Shooter motor over current!");
            outtakeMotor.setVelocity(0);
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Outtake Target Velocity", "%.0f ticks/sec", launchVelocity);
        linearOpMode.telemetry.addData("Outtake Actual Velocity", "%.0f ticks/sec", outtakeMotor.getVelocity());
        linearOpMode.telemetry.addData("Velocity Error", "%.0f ticks/sec", launchVelocity - outtakeMotor.getVelocity());
        linearOpMode.telemetry.addData("Outtake Current Draw", "%.2f A", outtakeMotor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
        linearOpMode.telemetry.addData("Outtake Position", outtakeMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Velocity %", "%.1f%%", (outtakeMotor.getVelocity() / TeleOpConstants.MAX_VELOCITY) * 100);
        linearOpMode.telemetry.addData("Motor Enabled", outtakeMotor.isMotorEnabled());
        linearOpMode.telemetry.addData("Motor PIDF", outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Container Motor Power", containerMotor.getPower());
        linearOpMode.telemetry.addData("Flap Servo Position", flapServo.getPosition());
    }
}