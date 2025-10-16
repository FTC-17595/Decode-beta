package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    private final DcMotor intakeSpin;
    private final LinearOpMode linearOpMode;

    public Intake(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.intakeSpin = linearOpMode.hardwareMap.dcMotor.get("intakeSpin");
    }

    public void configureMotorModes() {
        intakeSpin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSpin.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void intakeArtifactSlow(boolean intakeArtifactSlow) {
        if (intakeArtifactSlow) {
            intakeSpin.setPower(0.2);
            linearOpMode.sleep(2500);
        } else if (linearOpMode.isStopRequested()) {
            intakeSpin.setPower(0);
        }
    }

    public void intakeArtifact(boolean intakeArtifact) {
        if (intakeArtifact) {
            intakeSpin.setPower(0.8);
            linearOpMode.sleep(2500);
        } else if (linearOpMode.isStopRequested()) {
            intakeSpin.setPower(0);
        }
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Intake Motor Power", intakeSpin.getPower());
        linearOpMode.telemetry.update();
    }
}
