package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DriveTrainBackward extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(this);

        driveTrain.configureMotorModes();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            driveTrain.setMovePower(-0.5);

        }

    }
}
