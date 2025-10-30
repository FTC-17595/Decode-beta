package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoTelemetry {
    private LinearOpMode linearOpMode;
    GoBildaPinpointDriver odo;

    public void displayTelemetryAuto() {
        linearOpMode.telemetry.addData("x: ", odo.getPosX(DistanceUnit.MM));
        linearOpMode.telemetry.addData("y: ", odo.getPosY(DistanceUnit.MM));
    }
}