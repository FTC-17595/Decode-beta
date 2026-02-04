package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.MechanismSubsystem;

@Config
@Autonomous(name = "Pedro Blue Auto", group = "Pedro")
public class AutoBluePedro extends OpMode {

    private Follower follower;
    private MechanismSubsystem mechanisms;
    private ElapsedTime pathTimer = new ElapsedTime();
    private ElapsedTime actionTimer = new ElapsedTime();

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose shootPose = new Pose(4.7, 0, Math.toRadians(19));
    private final Pose pickupStartPose = new Pose(16.5, 5.0, Math.toRadians(90));
    private final Pose pickupEndPose = new Pose(25.6, 5.0, Math.toRadians(90));
    private final Pose parkPose = new Pose(8.0, 4.0, Math.toRadians(108));

    private PathChain toShootingSpot, toPickupStart, intakeSampleLine, returnToShoot, toPark;

    private int pathState = 0;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        mechanisms = new MechanismSubsystem(hardwareMap);

        buildPaths();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.update();
    }

    private void buildPaths() {
        toShootingSpot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(shootPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        toPickupStart = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPose), new Point(pickupStartPose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupStartPose.getHeading())
                .build();

        intakeSampleLine = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupStartPose), new Point(pickupEndPose)))
                .setLinearHeadingInterpolation(pickupStartPose.getHeading(), pickupEndPose.getHeading())
                .setZeroPowerAccelerationMultiplier(0.5) 
                .build();

        returnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickupEndPose), new Point(shootPose)))
                .setLinearHeadingInterpolation(pickupEndPose.getHeading(), shootPose.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(shootPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        pathTimer.reset();
        actionTimer.reset();
        pathState = 0;
        
        follower.followPath(toShootingSpot);
        mechanisms.startShooter(); 
    }

    @Override
    public void loop() {
        follower.update();
        
        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();

        switch (pathState) {
            case 0: 
                if (!follower.isBusy()) {
                    mechanisms.shootArtifact(); 
                    actionTimer.reset();
                    pathState = 1;
                }
                break;

            case 1: 
                if (actionTimer.milliseconds() > 800) { 
                    mechanisms.resetFlap();
                    follower.followPath(toPickupStart);
                    pathState = 2;
                }
                break;

            case 2: 
                if (!follower.isBusy()) {
                    mechanisms.setIntakePower(1.0);
                    follower.followPath(intakeSampleLine);
                    pathState = 3;
                }
                break;

            case 3: 
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 4;
                }
                break;
                
            case 4: 
                if (actionTimer.milliseconds() > 500) {
                    mechanisms.setIntakePower(0); 
                    mechanisms.startShooter();    
                    follower.followPath(returnToShoot);
                    pathState = 5;
                }
                break;

            case 5: 
                if (!follower.isBusy()) {
                    mechanisms.shootArtifact(); 
                    actionTimer.reset();
                    pathState = 6;
                }
                break;
                
            case 6: 
                 if (actionTimer.milliseconds() > 800) {
                    mechanisms.resetFlap();
                    mechanisms.stopShooter();
                    follower.followPath(toPark);
                    pathState = 7;
                 }
                 break;
                 
            case 7: 
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }
}
