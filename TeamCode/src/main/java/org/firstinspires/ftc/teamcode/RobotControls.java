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
    boolean shiftArtifactInContainer;
//    boolean rejectContainerArtifact;
    public void updateControls() {
        intakeArtifact = linearOpMode.gamepad1.a;
        flapArtifact = linearOpMode.gamepad1.b;
        shiftArtifactInContainer = linearOpMode.gamepad1.x;
//        rejectContainerArtifact = linearOpMode.gamepad1.x;
        rejectIntakeArtifact = linearOpMode.gamepad1.y;
        motorBrake = linearOpMode.gamepad1.left_trigger;
        shootArtifact = linearOpMode.gamepad1.right_trigger;
    }
}
