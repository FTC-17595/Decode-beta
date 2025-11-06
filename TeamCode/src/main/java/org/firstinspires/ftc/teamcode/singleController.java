package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class singleController {
    private final LinearOpMode linearOpMode;

    public singleController(LinearOpMode linearOpMode) {
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
    //    boolean shiftArtifactInContainer;
//    boolean rejectContainerArtifact;
    boolean celebrate;
    public void updateControls() {
        intakeArtifact = linearOpMode.gamepad1.a;
        flapArtifact = linearOpMode.gamepad1.b;
        switchLaunchPower = linearOpMode.gamepad1.x;
        rejectIntakeArtifact = linearOpMode.gamepad1.y;
        motorBrake = linearOpMode.gamepad1.left_trigger;
        shootArtifact = linearOpMode.gamepad1.right_trigger;
        celebrate = linearOpMode.gamepad1.back;
        increaseFactor = linearOpMode.gamepad1.dpad_up;
        decreaseFactor = linearOpMode.gamepad1.dpad_down;
//        autoShoot = linearOpMode.gamepad2.right_bumper;
//        shiftArtifactInContainer = linearOpMode.gamepad2.x;
//        rejectContainerArtifact = linearOpMode.gamepad2.x;
    }
}
