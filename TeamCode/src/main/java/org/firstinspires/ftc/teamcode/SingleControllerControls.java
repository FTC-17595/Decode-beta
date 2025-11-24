package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SingleControllerControls {
    private final LinearOpMode linearOpMode;
    private final FtcDashboard dashboard;

    public SingleControllerControls(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.dashboard = FtcDashboard.getInstance();
    }

    boolean intakeArtifact;
    boolean rejectIntakeArtifact;
    boolean flapArtifact;
    float shootArtifact;
    float motorBrake;
    boolean increaseFactor;
    boolean decreaseFactor;
    boolean switchLaunchPower;
    boolean celebrate;
    boolean autoShoot;
    boolean alignRobot;

    public void updateControls() {
        intakeArtifact = linearOpMode.gamepad2.a;
        flapArtifact = linearOpMode.gamepad2.b;
        switchLaunchPower = linearOpMode.gamepad2.x;
        rejectIntakeArtifact = linearOpMode.gamepad2.y;
        motorBrake = linearOpMode.gamepad2.left_trigger;
        shootArtifact = linearOpMode.gamepad2.right_trigger;
        celebrate = linearOpMode.gamepad2.back;
        increaseFactor = linearOpMode.gamepad2.dpad_up;
        decreaseFactor = linearOpMode.gamepad2.dpad_down;
        autoShoot = linearOpMode.gamepad2.right_bumper;
        alignRobot = linearOpMode.gamepad1.b;
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Intake (A)", intakeArtifact);
        linearOpMode.telemetry.addData("Reject Intake (Y)", rejectIntakeArtifact);
        linearOpMode.telemetry.addData("Flap (B)", flapArtifact);
        linearOpMode.telemetry.addData("Shoot Trigger", shootArtifact);
        linearOpMode.telemetry.addData("Motor Brake Trigger", motorBrake);
        linearOpMode.telemetry.addData("Increase Velocity (DPad Up)", increaseFactor);
        linearOpMode.telemetry.addData("Decrease Velocity (DPad Down)", decreaseFactor);
        linearOpMode.telemetry.addData("Switch Launch Power (X)", switchLaunchPower);
        linearOpMode.telemetry.addData("Celebrate (Back)", celebrate);
        linearOpMode.telemetry.addData("Auto Shoot (RB)", autoShoot);
        linearOpMode.telemetry.addData("Align Robot (gamepad1 B)", alignRobot);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("intakeArtifact", intakeArtifact);
        packet.put("rejectIntakeArtifact", rejectIntakeArtifact);
        packet.put("flapArtifact", flapArtifact);
        packet.put("shootTrigger", shootArtifact);
        packet.put("motorBrake", motorBrake);
        packet.put("increaseFactor", increaseFactor);
        packet.put("decreaseFactor", decreaseFactor);
        packet.put("switchLaunchPower", switchLaunchPower);
        packet.put("celebrate", celebrate);
        packet.put("autoShoot", autoShoot);
        packet.put("alignRobot", alignRobot);
        dashboard.sendTelemetryPacket(packet);
    }
}
