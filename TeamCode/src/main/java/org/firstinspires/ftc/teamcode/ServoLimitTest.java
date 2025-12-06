package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Limit Test")

public class ServoLimitTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("flapServo");

        servo.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentPosition = servo.getPosition();

            if (gamepad1.dpad_up) {
                servo.setPosition(currentPosition + 0.0001);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(currentPosition - 0.0001);
            }

            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();
        }
    }
}
