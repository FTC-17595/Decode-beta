package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import java.util.Map;
import java.util.ArrayList;
import java.util.List;

public class Webcam {
    private final String webcamName = "Webcam 1"; // MODIFY THIS BASED ON WEBCAM NAME

    private final LinearOpMode linearOpMode;
    private final RobotControls robotControls;
    private ColorBlobLocatorProcessor colorLocator;
    public static AprilTagProcessor aprilTag;
    private VisionPortal portal; // Don't make this final

    public Webcam(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.robotControls = new RobotControls(linearOpMode);
        this.colorLocator = createProcessor(false); // Initialize with a default color
        this.aprilTag = new AprilTagProcessor.Builder()
                .build();
        this.portal = new VisionPortal.Builder()
                .addProcessors(colorLocator, aprilTag)
                .setCameraResolution(new Size(640, 360))
                .setCamera(linearOpMode.hardwareMap.get(WebcamName.class, webcamName))
                .build();
    }

    // SUPPORTED COLORS: GREEN (TRUE), PURPLE (FALSE, DEFAULT)
    // Creates a colorProcessor every time the target color is changed
    private ColorBlobLocatorProcessor createProcessor(Boolean targetColor) {
        if (targetColor == true) {
            return new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_GREEN) // Use a predefined color match
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                    .setDrawContours(true)   // Show contours on the Stream Preview
                    .setBoxFitColor(0)       // Disable the drawing of rectangles
                    .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                    .setBlurSize(5)          // Smooth the transitions between different colors in image

                    // The following options have been added to fill in perimeter holes.
                    .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                    .setErodeSize(15)        // Shrink blobs back to original size
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                    .build();
        } else {
            return new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(ColorRange.ARTIFACT_PURPLE) // Use a predefined color match
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                    .setDrawContours(true)   // Show contours on the Stream Preview
                    .setBoxFitColor(0)       // Disable the drawing of rectangles
                    .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                    .setBlurSize(5)          // Smooth the transitions between different colors in image

                    // The following options have been added to fill in perimeter holes.
                    .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                    .setErodeSize(15)        // Shrink blobs back to original size
                    .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                    .build();
        }
    }

    public void setTargetColorRange(Boolean colorRange) {
        ColorBlobLocatorProcessor newProcessor = createProcessor(colorRange);

        this.portal.setProcessorEnabled(this.colorLocator, false);
        this.portal.setProcessorEnabled(newProcessor, true);
        this.colorLocator = newProcessor;
    }

    /* This function simply returns color blobs for processing
    Remember to use the above function to set the color of the artifact this function is looking for */
    public List<ColorBlobLocatorProcessor.Blob> getBlobs(ColorBlobLocatorProcessor colorLocator) {
        return colorLocator.getBlobs();
    }

    /* This function processes color blobs into a list of circles for usage in other functions (e.g. when moving to an artifact automatically)
    It is recommended to adjust values within the function based on lighting and other environmental factors */
    public List<Circle> getCircleFits(ColorBlobLocatorProcessor colorLocator) {
        List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs(colorLocator);
        List<Circle> circles = new ArrayList<>();

        /* Filter the blobs based on area and circularity, edit these values as needed
        Please note that the environment can throw off the accuracy of the filter (e.g. venue lighting) */
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, blobs);

        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            circles.add(b.getCircle());
        }

        return circles;
    }

    /* Comments added in this big massive blob of linear algebra to help you follow along
     * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     * WARNING: Basic algebra required to understand this, please do not change this without
     * knowledge of the math used */
    public static class EkfLocalizer {

        // State vector: [x, y, theta] (robot pose)
        private DMatrixRMaj x = new DMatrixRMaj(3, 1);
        private DMatrixRMaj P = CommonOps_DDRM.identity(3); // Covariance

        // Process noise (motion uncertainty)
        private final DMatrixRMaj Q = CommonOps_DDRM.identity(3);
        // Measurement noise (AprilTag detection uncertainty)
        private final DMatrixRMaj R = CommonOps_DDRM.identity(2); // For (x, y) measurement

        // AprilTag field layout: tag ID -> known world position
        private Map<Integer, double[]> aprilTagMap;

        public EkfLocalizer(Map<Integer, double[]> tagMap) {
            this.aprilTagMap = tagMap;

            // Set initial uncertainties
            CommonOps_DDRM.scale(0.01, P);
            CommonOps_DDRM.scale(0.01, Q);
            CommonOps_DDRM.scale(0.5, R);  // High R → trust AprilTags less
        }

        // EKF prediction step using differential drive odometry
        public void predict(double linearVel, double angularVel, double dt) {
            double theta = x.get(2, 0);

            // Predict new state
            double dx = linearVel * dt * Math.cos(theta);
            double dy = linearVel * dt * Math.sin(theta);
            double dTheta = angularVel * dt;

            x.add(0, 0, dx);
            x.add(1, 0, dy);
            x.add(2, 0, dTheta);

            // Normalize angle
            x.set(2, 0, normalizeAngle(x.get(2, 0)));

            // Compute Jacobian of motion model w.r.t. state (F)
            DMatrixRMaj F = new DMatrixRMaj(3, 3);
            F.set(0, 0, 1);
            F.set(0, 2, -linearVel * dt * Math.sin(theta));
            F.set(1, 1, 1);
            F.set(1, 2, linearVel * dt * Math.cos(theta));
            F.set(2, 2, 1);

            // P = F * P * F^T + Q
            DMatrixRMaj temp = new DMatrixRMaj(3, 3);
            CommonOps_DDRM.mult(F, P, temp);
            CommonOps_DDRM.multTransB(temp, F, P);
            CommonOps_DDRM.addEquals(P, Q);
        }

        // EKF update step using AprilTag detection
        public void updateWithAprilTag(int tagID, double relX, double relY, double thetaCameraToRobot) {
            double[] tagPos = aprilTagMap.get(tagID);
            if (tagPos == null) return;

            double tagX = tagPos[0];
            double tagY = tagPos[1];

            // Predict where the robot would be if the tag is at (tagX, tagY) and the robot sees it at (relX, relY)
            // World robot pose = tagWorld - rotated relative offset
            double theta = x.get(2, 0);
            double worldX = tagX - (Math.cos(theta + thetaCameraToRobot) * relX - Math.sin(theta + thetaCameraToRobot) * relY);
            double worldY = tagY - (Math.sin(theta + thetaCameraToRobot) * relX + Math.cos(theta + thetaCameraToRobot) * relY);

            DMatrixRMaj z = new DMatrixRMaj(2, 1);
            z.set(0, 0, worldX);
            z.set(1, 0, worldY);

            // h(x) = [x; y]
            DMatrixRMaj h = new DMatrixRMaj(2, 1);
            h.set(0, 0, x.get(0, 0));
            h.set(1, 0, x.get(1, 0));

            // Innovation: y = z - h(x)
            DMatrixRMaj y = new DMatrixRMaj(2, 1);
            CommonOps_DDRM.subtract(z, h, y);

            // Measurement Jacobian H
            DMatrixRMaj H = new DMatrixRMaj(2, 3);
            H.set(0, 0, 1); // ∂h/∂x
            H.set(0, 1, 0);
            H.set(0, 2, 0);
            H.set(1, 0, 0);
            H.set(1, 1, 1); // ∂h/∂y
            H.set(1, 2, 0);

            // S = HPH^T + R
            DMatrixRMaj S = new DMatrixRMaj(2, 2);
            DMatrixRMaj HPHt = new DMatrixRMaj(2, 2);
            CommonOps_DDRM.mult(H, P, HPHt);
            CommonOps_DDRM.multTransB(HPHt, H, S);
            CommonOps_DDRM.addEquals(S, R);

            // K = PH^T S⁻¹
            DMatrixRMaj K = new DMatrixRMaj(3, 2);
            DMatrixRMaj PHt = new DMatrixRMaj(3, 2);
            CommonOps_DDRM.multTransB(P, H, PHt);
            DMatrixRMaj S_inv = new DMatrixRMaj(2, 2);
            CommonOps_DDRM.invert(S, S_inv);
            CommonOps_DDRM.mult(PHt, S_inv, K);

            // x = x + Ky
            DMatrixRMaj Ky = new DMatrixRMaj(3, 1);
            CommonOps_DDRM.mult(K, y, Ky);
            CommonOps_DDRM.addEquals(x, Ky);
            x.set(2, 0, normalizeAngle(x.get(2, 0)));

            // P = (I - KH)P
            DMatrixRMaj I = CommonOps_DDRM.identity(3);
            DMatrixRMaj KH = new DMatrixRMaj(3, 3);
            CommonOps_DDRM.mult(K, H, KH);
            DMatrixRMaj IminusKH = new DMatrixRMaj(3, 3);
            CommonOps_DDRM.subtract(I, KH, IminusKH);
            CommonOps_DDRM.mult(IminusKH, P, P);
        }

        public double[] getEstimatedPose() {
            return new double[]{
                    x.get(0, 0), // x
                    x.get(1, 0), // y
                    x.get(2, 0)  // theta
            };
        }

        private double normalizeAngle(double angle) {
            return Math.atan2(Math.sin(angle), Math.cos(angle));
        }
    }

    public static class AprilTagDetector {
        private final AprilTagProcessor aprilTagDetection = aprilTag;

        public AprilTagDetector(AprilTagProcessor aprilTag) {
            aprilTagDetector =

        }

        public AprilTagDetection getLatestDetection() {
            return aprilTag.getDetections();
        }

    }

    // Call this function at the end of the OpMode to stop the camera safely
    public void closeStream(VisionPortal portal) {
        if (portal != null) {
            portal.close();
        }
    }
}
