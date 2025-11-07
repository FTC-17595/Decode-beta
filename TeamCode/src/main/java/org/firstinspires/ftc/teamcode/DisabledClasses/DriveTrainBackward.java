package org.firstinspires.ftc.teamcode.DisabledClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.DriveTrain;

@Autonomous
@Disabled
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
