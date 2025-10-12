package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Motor Run")
public class Motor_Run extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor outtakeMotor = hardwareMap.dcMotor.get("outtakeMotor");
        final IMU imu;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            outtakeMotor.setPower(gamepad2.right_trigger);
        }
    }
}
