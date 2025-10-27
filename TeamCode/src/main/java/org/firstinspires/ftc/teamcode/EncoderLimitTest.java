package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Encoder Limit Test")
public class EncoderLimitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor dcMotor = hardwareMap.dcMotor.get("outtakeMotor");
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        double speed = 0.0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            telemetry.addData("Speed", speed);
            telemetry.addData("Motor Power", dcMotor.getPower());
            telemetry.update();
        }
    }
}
