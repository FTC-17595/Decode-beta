package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {

    private final DcMotor intakeSpin;
    private final LinearOpMode linearOpMode;

    public Intake(LinearOpMode linearOpMode) {
        this.intakeSpin = linearOpMode.hardwareMap.dcMotor.get("intakeSpin");
        this.linearOpMode = linearOpMode;

    }

    public void configureMotorModes() {
        intakeSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeSpin.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void inttakeArtifact(boolean intakeArtifact) {
        if (inttakeArtifact) {
            intakeSpin.setPower(0.8);
            sleep(2500);
        } else {
             intakeSpin.setPower(0);
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Intake Motor Power", intakeSpin.getPower());
        linearOpMode.telemetry.update();

    
    }
}
