package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.teamcode.TeleOpConstants.MAX_VELOCITY;

@TeleOp(name = "Encoder Limit Test")
@Disabled
public class EncoderLimitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx dcMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        TeleOpConstants TeleOpConstants;
        // Configure motor for velocity control
        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double speed = 0.0;
        double targetVelocity = 0.0;
        boolean useVelocityMode = false;

        telemetry.addLine("Press X to toggle between Power and Velocity mode");
        telemetry.addLine("D-Pad Up/Down to adjust");
        telemetry.addLine("A to run motor");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Toggle between power and velocity mode
            if (gamepad1.x) {
                useVelocityMode = !useVelocityMode;
                sleep(200); // Debounce
            }

            if (useVelocityMode) {
                // Velocity mode adjustments
                if (gamepad1.dpad_up) {
                    targetVelocity += TeleOpConstants.VELOCITY_INCREMENT;
                } else if (gamepad1.dpad_down) {
                    targetVelocity -= TeleOpConstants.VELOCITY_INCREMENT;
                }

                // Clamp velocity to safe limits
                targetVelocity = Math.max(-TeleOpConstants.MAX_VELOCITY, Math.min(TeleOpConstants.MAX_VELOCITY, targetVelocity));

                if (gamepad1.a) {
                    dcMotor.setVelocity(targetVelocity);
                } else {
                    dcMotor.setVelocity(0);
                }
            } else {
                // Power mode adjustments
                if (gamepad1.dpad_up) {
                    speed += 0.1;
                } else if (gamepad1.dpad_down) {
                    speed -= 0.1;
                }

                // Clamp speed between -1 and 1
                speed = Math.max(-1.0, Math.min(1.0, speed));

                if (gamepad1.a) {
                    dcMotor.setPower(speed);
                } else {
                    dcMotor.setPower(0);
                }
            }

            // Comprehensive telemetry (similar to ArtifactHandlingSystem)
            telemetry.addLine("=== CONTROL MODE ===");
            telemetry.addData("Mode", useVelocityMode ? "VELOCITY" : "POWER");
            telemetry.addLine();

            if (useVelocityMode) {
                telemetry.addLine("=== VELOCITY CONTROL ===");
                telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
                telemetry.addData("Actual Velocity", "%.0f ticks/sec", dcMotor.getVelocity());
                telemetry.addData("Velocity Error", "%.0f ticks/sec", targetVelocity - dcMotor.getVelocity());
                telemetry.addData("Velocity %", "%.1f%%", (dcMotor.getVelocity() / TeleOpConstants.MAX_VELOCITY) * 100);
            } else {
                telemetry.addLine("=== POWER CONTROL ===");
                telemetry.addData("Target Power", "%.2f", speed);
                telemetry.addData("Actual Power", "%.2f", dcMotor.getPower());
                telemetry.addData("Estimated Velocity", "%.0f ticks/sec", dcMotor.getVelocity());
            }

            telemetry.addLine();
            telemetry.addLine("=== MOTOR STATUS ===");
            telemetry.addData("Encoder Position", dcMotor.getCurrentPosition());
            telemetry.addData("Current Draw", "%.2f A", dcMotor.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS));
            telemetry.addData("Motor Enabled", dcMotor.isMotorEnabled());
            telemetry.addData("Over Current", dcMotor.isOverCurrent() ? "⚠️ YES" : "No");

            telemetry.addLine();
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addData("X", "Toggle Mode");
            telemetry.addData("D-Pad Up/Down", useVelocityMode ? "Adjust Velocity" : "Adjust Power");
            telemetry.addData("A (Hold)", "Run Motor");

            telemetry.update();
        }
    }
}