package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class ArtifactHandlingSystem {

    private final DcMotorEx outtakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor containerMotor;
    private final Servo flapServo;
    private final LinearOpMode linearOpMode;
    private final FtcDashboard dashboard;
    private double launchVelocity;
    private double autoLaunchVelocity;
    private boolean lastSwitchState = false;
    private boolean flapDown = true;
    private boolean autoFeeding = false;
    private String intakeAutoStatus = "idle";
    private boolean intakeStoppedForCapacity = false;

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.dcMotor.get("containerMotor");
        this.flapServo = linearOpMode.hardwareMap.servo.get("flapServo");
        this.linearOpMode = linearOpMode;
        this.dashboard = FtcDashboard.getInstance();
    }

    public void configureMotorModes() {
        MotorConfigurationType config = outtakeMotor.getMotorType().clone();
        config.setAchieveableMaxRPMFraction(1.0);
        outtakeMotor.setMotorType(config);

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);

        launchVelocity = TeleOpConstants.EXTRA_LONG_RANGE_VELOCITY;
        autoLaunchVelocity = TeleOpConstants.AUTO_LONG_RANGE_VELOCITY;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                TeleOpConstants.kP,
                TeleOpConstants.kI,
                TeleOpConstants.kD,
                TeleOpConstants.kF
        );

        outtakeMotor.setVelocityPIDFCoefficients(TeleOpConstants.kP, TeleOpConstants.kI,TeleOpConstants.kD, TeleOpConstants.kF);
        outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void intakeSystem(boolean intakeArtifact, boolean rejectArtifact, int artifactCount) {
        intakeStoppedForCapacity = false;
        if (artifactCount >= TeleOpConstants.MAX_ARTIFACT_CAPACITY && intakeArtifact) {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
            intakeAutoStatus = "stopped: full (>=cap)";
            intakeStoppedForCapacity = true;
            return;
        }

        if (intakeArtifact) {
            intakeMotor.setPower(1);
            containerMotor.setPower(1);
            intakeAutoStatus = "intake: forward";
        } else if (rejectArtifact) {
            intakeMotor.setPower(-1);
            containerMotor.setPower(-1);
            intakeAutoStatus = "intake: reverse";
        } else {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
            intakeAutoStatus = "intake: idle";
        }
    }

    public void flapSystem(boolean flapUp) {
        if (flapUp) {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_UP);
            flapDown = false;
        } else {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);
            flapDown = true;
        }
    }
    
    public void manageIntakeWithAutoFeed(boolean intakeArtifact, boolean rejectArtifact, boolean shooterActive, boolean artifactAtBack, int artifactCount) {
        if (intakeArtifact || rejectArtifact) {
            autoFeeding = false;
            intakeAutoStatus = intakeArtifact ? "manual: intake" : "manual: reject";
            intakeSystem(intakeArtifact, rejectArtifact, artifactCount);
            return;
        }

        if (shooterActive && flapDown && !artifactAtBack) {
            autoFeeding = true;
            intakeAutoStatus = "auto: feeding (shoot)";
            intakeSystem(true, false, artifactCount);
            return;
        }

        if (autoFeeding) {
            intakeSystem(false, false, artifactCount);
            autoFeeding = false;
            intakeAutoStatus = "auto: stopped";
        } else {
            intakeSystem(false, false, artifactCount);
            intakeAutoStatus = "idle";
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
        });

        ArtifactShoot.start();
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0.1) {
            outtakeMotor.setVelocity(launchVelocity);
        } else if (rejectArtifact > 0.1) {
            outtakeMotor.setVelocity(-500);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public void updateLaunchVelocityForRange(double rangeInches, double tagId) {
        if (Double.isNaN(rangeInches) || rangeInches <= 0) {
            return;
        }

        if (rangeInches >= TeleOpConstants.ALIGN_EXTRA_LONG_RANGE_MIN_IN // EXTRA LONG
                && rangeInches <= TeleOpConstants.ALIGN_EXTRA_LONG_RANGE_MAX_IN) {
            if (tagId == 20) { // BLUE
                launchVelocity = TeleOpConstants.EXTRA_LONG_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_LONG_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 50 / 25.4;
            } else { // RED
                launchVelocity = TeleOpConstants.EXTRA_LONG_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_LONG_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 50 / 25.4;
            }
        } else if (rangeInches >= TeleOpConstants.ALIGN_LONG_RANGE_MIN_IN // LONG
                && rangeInches <= TeleOpConstants.ALIGN_LONG_RANGE_MAX_IN) {
            if (tagId == 20) { // BLUE
                launchVelocity = TeleOpConstants.LONG_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_LONG_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            } else { // RED
                launchVelocity = TeleOpConstants.LONG_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            }
        } else if (rangeInches >= TeleOpConstants.ALIGN_MEDIUM_RANGE_MIN_IN // MEDIUM
                && rangeInches <= TeleOpConstants.ALIGN_MEDIUM_RANGE_MAX_IN) {
            if (tagId == 20) { // BLUE
                launchVelocity = TeleOpConstants.MEDIUM_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            } else { // RED
                launchVelocity = TeleOpConstants.MEDIUM_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            }
        } else { // SHORT
            if (tagId == 20) { // BLUE
                launchVelocity = TeleOpConstants.SHORT_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            } else { // RED
                launchVelocity = TeleOpConstants.SHORT_RANGE_VELOCITY;
                autoLaunchVelocity = TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 0 / 25.4;
            }
        }
    }

    private void waitForMotor() {
        while (outtakeMotor.getVelocity() > autoLaunchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                outtakeMotor.getVelocity() < autoLaunchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                linearOpMode.opModeIsActive()) {

            linearOpMode.telemetry.addData("Outtake Velocity", outtakeMotor.getVelocity());
            linearOpMode.telemetry.addData("Outtake Target Velocity", outtakeMotor.getTargetPosition());
            linearOpMode.telemetry.addData("Launch Velocity", autoLaunchVelocity);
            linearOpMode.telemetry.addData("Velocity -", outtakeMotor.getVelocity() > autoLaunchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);
            linearOpMode.telemetry.addData("Velocity +", outtakeMotor.getVelocity() < autoLaunchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);
            linearOpMode.telemetry.addData("Velocity T/F", outtakeMotor.getVelocity() > autoLaunchVelocity - TeleOpConstants.WAIT_FOR_MOTOR_OFFSET &&
                    outtakeMotor.getVelocity() < autoLaunchVelocity + TeleOpConstants.WAIT_FOR_MOTOR_OFFSET);

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
        // Max theoretical = 2,300 ticks/sec;
        launchVelocity = Math.max(0, Math.min(launchVelocity, TeleOpConstants.MAX_VELOCITY));
    }

    public void switchShootingFactor(boolean switch_f) {
        // Only toggle when switch_f transitions from false to true (rising edge)
        if (switch_f && !lastSwitchState) {
            launchVelocity = (launchVelocity == TeleOpConstants.SHORT_RANGE_VELOCITY)
                    ? TeleOpConstants.LONG_RANGE_VELOCITY
                    : TeleOpConstants.SHORT_RANGE_VELOCITY;

            autoLaunchVelocity = (autoLaunchVelocity == TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY)
                    ? TeleOpConstants.AUTO_LONG_RANGE_VELOCITY
                    : TeleOpConstants.AUTO_SHORT_RANGE_VELOCITY;
        }
        lastSwitchState = switch_f;
    }

    public boolean isFlapDown() {
        return flapDown;
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
            linearOpMode.telemetry.update();
            outtakeMotor.setVelocity(0);
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Outtake Target Velocity", "%.0f ticks/sec", launchVelocity);
        linearOpMode.telemetry.addData("Outtake AUTO Target Velocity", "%.0f ticks/sec", autoLaunchVelocity);
        linearOpMode.telemetry.addData("Outtake Actual Velocity", "%.0f ticks/sec", outtakeMotor.getVelocity());
        linearOpMode.telemetry.addData("Velocity Error", "%.0f ticks/sec", launchVelocity - outtakeMotor.getVelocity());
        linearOpMode.telemetry.addData("AUTO Velocity Error", "%.0f ticks/sec", autoLaunchVelocity - outtakeMotor.getVelocity());
        linearOpMode.telemetry.addData("Outtake Current Draw", "%.2f A", outtakeMotor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
        linearOpMode.telemetry.addData("Outtake Position", outtakeMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Velocity %", "%.1f%%", (outtakeMotor.getVelocity() / TeleOpConstants.MAX_VELOCITY) * 100);
        linearOpMode.telemetry.addData("Motor Enabled", outtakeMotor.isMotorEnabled());
        linearOpMode.telemetry.addData("Motor PIDF", outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Container Motor Power", containerMotor.getPower());
        linearOpMode.telemetry.addData("Flap Servo Position", flapServo.getPosition());
        linearOpMode.telemetry.addData("Intake Auto Status", intakeAutoStatus);
        linearOpMode.telemetry.addData("Intake Auto-Stopped (Full)", intakeStoppedForCapacity);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVelocity", launchVelocity);
        packet.put("autoTargetVelocity", autoLaunchVelocity);
        packet.put("actualVelocity", outtakeMotor.getVelocity());
        packet.put("velocityError", launchVelocity - outtakeMotor.getVelocity());
        packet.put("autoVelocityError", autoLaunchVelocity - outtakeMotor.getVelocity());
        packet.put("intakePower", intakeMotor.getPower());
        packet.put("containerPower", containerMotor.getPower());
        packet.put("flapPosition", flapServo.getPosition());
        packet.put("intakeAutoStatus", intakeAutoStatus);
        packet.put("intakeAutoStopped", intakeStoppedForCapacity);
        dashboard.sendTelemetryPacket(packet);
    }
}
