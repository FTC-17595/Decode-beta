package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotControls {
    private final LinearOpMode linearOpMode;

    public RobotControls(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    boolean intakeArtifact;
    boolean rejectIntakeArtifact;
    float shootArtifact;
    float rejectShootingArtifact;
    boolean shiftArtifactInContainer;
    boolean rejectContainerArtifact;
    public void updateControls() {
        intakeArtifact = linearOpMode.gamepad1.a;
        shiftArtifactInContainer = linearOpMode.gamepad1.b;
        rejectContainerArtifact = linearOpMode.gamepad1.x;
        rejectIntakeArtifact = linearOpMode.gamepad1.y;
        rejectShootingArtifact = linearOpMode.gamepad1.left_trigger;
        shootArtifact = linearOpMode.gamepad1.right_trigger;
    }
}
