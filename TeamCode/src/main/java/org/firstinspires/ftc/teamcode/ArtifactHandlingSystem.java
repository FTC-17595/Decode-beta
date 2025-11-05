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

    // Define target velocities in ticks per second
    // RS-555 Motor Specs:
    // - 6000 RPM no-load speed
    // - 28 PPR encoder with quadrature (x4) = 112 CPR (counts per revolution)
    // - Gear ratio: 1:1 (direct drive)
    //
    // Max theoretical velocity calculation:
    // 6000 RPM ÷ 60 = 100 rev/sec
    // 100 rev/sec × 112 ticks/rev = 11,200 ticks/sec
    //
    // Recommended operating range: 70-85% of max for PID stability and load handling

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
    } /* 😟😟😟 */

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setVelocity(launchVelocity * shootArtifact);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setVelocity(-launchVelocity * rejectArtifact);
        } else {
            outtakeMotor.setVelocity(0);
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
        if (switch_f) {
            launchVelocity = (launchVelocity == TeleOpConstants.SHORT_RANGE_VELOCITY) ? TeleOpConstants.LONG_RANGE_VELOCITY : TeleOpConstants.SHORT_RANGE_VELOCITY;
        }
    }

    public void shootingSystemAuto(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setVelocity(AutoConstants.AUTO_ARTIFACT_SHOOT_VELOCITY);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setVelocity(-launchVelocity * rejectArtifact);
        } else {
            outtakeMotor.setVelocity(0);
        }
    }

    public void checkMotorHealth() {
        if (outtakeMotor.isOverCurrent()) {
            linearOpMode.telemetry.addData("⚠️ WARNING", "Shooter motor over current!");
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
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Container Motor Power", containerMotor.getPower());
        linearOpMode.telemetry.addData("Flap Servo Position", flapServo.getPosition());
        linearOpMode.telemetry.addData("Motor Enabled", outtakeMotor.isMotorEnabled());
    }
}