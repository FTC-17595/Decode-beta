package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// This is just a test, I apologize for it being quite sloppy
@Autonomous(name = "Intake Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure to use test motor configurations or modify the line below
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        waitForStart();
        if (isStopRequested()) return;

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            intakeMotor.setPower(0.9);
            telemetry.addData("Motor Direction", intakeMotor.getDirection());
            telemetry.addData("Intake Motor Info", intakeMotor.getConnectionInfo());
            telemetry.update();
        }

        intakeMotor.setPower(0);
    }
}