package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotControls {
    private final LinearOpMode linearOpMode;

    public RobotControls(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
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
    }
}
