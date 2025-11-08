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

    public void autoShootingSystemTeleOp(boolean autoShoot) {
        if (autoShoot) {
            shootingSystem(1f, 0f);
            linearOpMode.sleep(TeleOpConstants.SPINUP_MS);

            if (prepareArtifact(false)) {
                return;
            }

            fireArtifact();

            for (int i = 1; i < 4 && linearOpMode.opModeIsActive(); i++) {
                if (prepareArtifact(true)) {
                    break;
                }
                fireArtifact();
            }

            shootingSystem(0f, 0f);
            intakeSystem(false, false);
            flapSystem(false);
        }
    }

    private boolean prepareArtifact(boolean useFeed) {
        if (!linearOpMode.opModeIsActive()) {
            return true;
        }

        if (useFeed) {
            intakeSystem(true, false);
            linearOpMode.sleep(TeleOpConstants.FEED_MS);
            intakeSystem(false, false);
        } /* else { */
//            linearOpMode.sleep(TeleOpConstants.FEED_MS);
//        }

        if (!linearOpMode.opModeIsActive()) {
            return true;
        }

        linearOpMode.sleep(TeleOpConstants.FEED_SETTLE_MS);

        if (!linearOpMode.opModeIsActive()) {
            return true;
        }

        linearOpMode.sleep(TeleOpConstants.PREFIRE_WAIT_MS);
        return !linearOpMode.opModeIsActive();
    }

    private void fireArtifact() {
        if (!linearOpMode.opModeIsActive()) {
            return;
        }
        waitForMotor();
        flapSystem(true);
        linearOpMode.sleep(TeleOpConstants.SERVO_UP_MS);
        flapSystem(false);
        linearOpMode.sleep(TeleOpConstants.SERVO_RESET_MS);
    }

    private void waitForMotor() {
        while (outtakeMotor.getVelocity() > launchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                outtakeMotor.getVelocity() < launchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                linearOpMode.opModeIsActive()) {

            linearOpMode.telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
            linearOpMode.telemetry.addData("Outtake Target Velocity", outtakeMotor.getTargetPosition());
            linearOpMode.telemetry.addData("Launch Velocity", launchVelocity);
            linearOpMode.telemetry.addData("Velocity -", outtakeMotor.getVelocity() > launchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);
            linearOpMode.telemetry.addData("Velocity +", outtakeMotor.getVelocity() < launchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);
            linearOpMode.telemetry.addData("Velocity T/F", outtakeMotor.getVelocity() > launchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                    outtakeMotor.getVelocity() < launchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);

            linearOpMode.telemetry.update();
            linearOpMode.sleep(10);
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