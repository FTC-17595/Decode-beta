package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;  // Pinpoint uses this class :contentReference[oaicite:1]{index=1}
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name="EKF + Pinpoint + AprilTag", group="Localization")
public class EkfAutonomous extends LinearOpMode {

    private Webcam.EkfLocalizer localizer;
    private GoBildaPinpointDriver pinpoint;
    private Webcam.AprilTagDetector tagDetector;

    // Previous odometry pose values
    private double lastX = 0.0;
    private double lastY = 0.0;
    private double lastTheta = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- 1. Tag world‑map (IDs → world coordinates, in your chosen units: meters) ---
        Map<Integer,double[]> tagMap = new HashMap<>();
        tagMap.put(1, new double[]{2.0, 1.5});
        // add other tag entries…

        // --- 2. Initialize EKF localizer ---
        localizer = new Webcam.EkfLocalizer(tagMap);

        // --- 3. Initialize hardware / drivers ---
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "Pinpoint");  // ensure name matches your config
        pinpoint.initialize();  // or whatever init method exists

        tagDetector = new Webcam.AprilTagDetector(Webcam.aprilTag);
        tagDetector.initialize();

        telemetry.addLine("Initialized; waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- 4. Read initial pose from Pinpoint and set EKF state ---
        Pose2D initPose = pinpoint.getPosition();
        double xInitInches = initPose.getX(DistanceUnit.INCH);
        double yInitInches = initPose.getY(DistanceUnit.INCH);
        double thetaInitRad = initPose.getHeading(AngleUnit.RADIANS);

        // Convert inches → meters (assuming your EKF uses meters)
        double xInit = xInitInches * 0.0254;
        double yInit = yInitInches * 0.0254;

        localizer. (xInit, yInit, thetaInitRad);
        lastX = xInit;
        lastY = yInit;
        lastTheta = thetaInitRad;

        double lastTime = getRuntime();

        // --- 5. Main loop ---
        while (opModeIsActive() && !isStopRequested()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // Read current pose from Pinpoint
            Pose2D currentPose = pinpoint.getPosition();
            double xInches = currentPose.getX(DistanceUnit.INCH);
            double yInches = currentPose.getY(DistanceUnit.INCH);
            double thetaRad = currentPose.getHeading(AngleUnit.RADIANS);

            double xMeters = xInches * 0.0254;
            double yMeters = yInches * 0.0254;

            // Compute difference (pose change) – approximate velocity
            double dx = xMeters - lastX;
            double dy = yMeters - lastY;
            double dTheta = normalizeAngle(thetaRad - lastTheta);

            double linearVel = Math.hypot(dx, dy) / dt;
            double angularVel = dTheta / dt;

            lastX = xMeters;
            lastY = yMeters;
            lastTheta = thetaRad;

            // EKF predict step
            localizer.predict(linearVel, angularVel, dt);

            // AprilTag measurement check
            AprilTagDetection det = tagDetector.getLatestDetection();
            if (det != null && det.id != -1) {
                localizer.updateWithAprilTag(det.id, det.relX, det.relY, det.cameraToRobotHeading);
            }

            // Get and display estimated pose
            double[] est = localizer.getEstimatedPose();
            telemetry.addData("Est X (m)", String.format("%.2f", est[0]));
            telemetry.addData("Est Y (m)", String.format("%.2f", est[1]));
            telemetry.addData("Est θ (rad)", String.format("%.2f", est[2]));
            telemetry.addData("Pinpoint X (m)", String.format("%.2f", xMeters));
            telemetry.addData("Pinpoint Y (m)", String.format("%.2f", yMeters));
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}

