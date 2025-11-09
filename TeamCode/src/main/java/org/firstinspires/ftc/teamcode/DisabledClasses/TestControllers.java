package org.firstinspires.ftc.teamcode.DisabledClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test Controllers")
@Disabled
public class TestControllers extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor motor = hardwareMap.dcMotor.get("outtakeMotor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            telemetry.addData("G2RTR", gamepad2.right_trigger);
            telemetry.addData("G2LTR", gamepad2.left_trigger);
            telemetry.update();

            if (gamepad2.a) {
                motor.setPower(1.0);
            } else if (gamepad2.b) {
                motor.setPower(-1.0);
            } else {
                motor.setPower(0.0);
            }
        }
    }
}
