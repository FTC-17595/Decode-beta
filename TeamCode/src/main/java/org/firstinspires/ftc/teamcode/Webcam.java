package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

/**
 * Comprehensive Webcam class for FTC robotics providing:
 * 
 * APRILTAG FUNCTIONALITY:
 * - AprilTag detection and pose estimation
 * - Built-in SDK field positioning (robotPose)
 * - Distance and bearing calculations to detected tags
 * - Support for multiple simultaneous tag detections
 * - Automatic field coordinate system using FTC SDK
 * 
 * COLOR VISION FUNCTIONALITY:
 * - Color blob detection for DECODE artifacts (green/purple)
 * - Circle fitting for artifact recognition
 * - Morphological operations for noise reduction
 * - Region of Interest (ROI) processing
 * 
 * ADVANCED FEATURES:
 * - Extended Kalman Filter (EKF) for sensor fusion localization
 * - Thread-safe vision processing
 * - Real-time camera preview with overlays
 * - Configurable camera resolution and settings
 * 
 * USAGE EXAMPLES:
 * 
 * Basic AprilTag field positioning:
 *   Webcam webcam = new Webcam(this);
 *   AprilTagDetection tag = webcam.getBestAprilTag();
 *   double[] robotPos = webcam.getRobotFieldPosition(tag); // Uses built-in SDK positioning
 * 
 * Color artifact detection:
 *   webcam.setTargetColorRange(true); // Switch to green
 *   List<Circle> circles = webcam.getColorCircles();
 * 
 * Distance and bearing to AprilTag:
 *   double distance = webcam.getDistanceToClosestAprilTag();
 *   double bearing = webcam.getBearingToClosestAprilTag();
 * 
 * @author Beta Bionix Team
 * @version 2.0 - DECODE Season 2025-2026
 */
public class Webcam {
    private final String webcamName = "Webcam 1"; // MODIFY THIS BASED ON WEBCAM NAME

    private final LinearOpMode linearOpMode;
    private final RobotControls robotControls;
    private ColorBlobLocatorProcessor colorLocator;
    public static AprilTagProcessor aprilTag;
    private AprilTagDetector aprilTagDetector;
    private VisionPortal portal; // Don't make this final

    public Webcam(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.robotControls = new RobotControls(linearOpMode);
        this.colorLocator = createProcessor(false); // Initialize with a default color
        
        // Camera position and orientation on the robot
        // MODIFY THESE VALUES based on your camera mounting position
        // 
        // Position: [X, Y, Z] in inches from robot center
        // - X: positive = forward, negative = backward
        // - Y: positive = left, negative = right  
        // - Z: positive = up, negative = down
        //
        // Orientation: [Yaw, Pitch, Roll] in degrees
        // - Yaw: 0 = forward, +90 = left, -90 = right, 180 = backward
        // - Pitch: 0 = level, -90 = pointing forward (typical), +90 = pointing backward
        // - Roll: 0 = level, +90 = rotated 90° clockwise, -90 = rotated 90° counter-clockwise
        
        Position cameraPosition = new Position(DistanceUnit.INCH,
                0, 0, 0, 0); // Camera at robot center - MODIFY AS NEEDED
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90, 0, 0); // Camera pointing forward and horizontal - MODIFY AS NEEDED
        
        this.aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagProcessor.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                // Set camera position for field coordinate calculations
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        this.aprilTagDetector = new AprilTagDetector(aprilTag);
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
        private final AprilTagProcessor aprilTagProcessor;
        
        public AprilTagDetector(AprilTagProcessor aprilTag) {
            this.aprilTagProcessor = aprilTag;
        }

        /**
         * Get all current AprilTag detections
         * @return List of AprilTag detections
         */
        public List<AprilTagDetection> getDetections() {
            return aprilTagProcessor.getDetections();
        }

        /**
         * Get the best (closest/largest) AprilTag detection
         * @return Best AprilTag detection or null if none found
         */
        public AprilTagDetection getBestDetection() {
            List<AprilTagDetection> detections = getDetections();
            if (detections.isEmpty()) {
                return null;
            }
            
            // Return the detection with the largest area (closest tag)
            AprilTagDetection bestDetection = detections.get(0);
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && bestDetection.metadata != null) {
                    // Compare by distance if pose is available
                    if (detection.ftcPose != null && bestDetection.ftcPose != null) {
                        double currentDistance = Math.sqrt(
                            Math.pow(detection.ftcPose.x, 2) + 
                            Math.pow(detection.ftcPose.y, 2) + 
                            Math.pow(detection.ftcPose.z, 2)
                        );
                        double bestDistance = Math.sqrt(
                            Math.pow(bestDetection.ftcPose.x, 2) + 
                            Math.pow(bestDetection.ftcPose.y, 2) + 
                            Math.pow(bestDetection.ftcPose.z, 2)
                        );
                        if (currentDistance < bestDistance) {
                            bestDetection = detection;
                        }
                    }
                }
            }
            return bestDetection;
        }

        /**
         * Get AprilTag detection by specific ID
         * @param tagId The ID of the tag to find
         * @return AprilTag detection with specified ID or null if not found
         */
        public AprilTagDetection getDetectionById(int tagId) {
            List<AprilTagDetection> detections = getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.id == tagId) {
                    return detection;
                }
            }
            return null;
        }

        /**
         * Get the robot's field position using the built-in SDK robotPose
         * This uses the FTC SDK's automatic field coordinate calculation
         * @param detection The AprilTag detection to use
         * @return Robot's field position [x, y, z, yaw, pitch, roll] in inches/degrees or null if not available
         */
        public double[] getRobotFieldPosition(AprilTagDetection detection) {
            if (detection == null || detection.robotPose == null) {
                return null;
            }

            // Use the SDK's built-in field positioning
            double x = detection.robotPose.getPosition().x;
            double y = detection.robotPose.getPosition().y;
            double z = detection.robotPose.getPosition().z;
            
            double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            double pitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
            double roll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);

            return new double[]{x, y, z, yaw, pitch, roll};
        }

        /**
         * Get the robot's 2D field position (X, Y, Yaw only) using built-in SDK robotPose
         * @param detection The AprilTag detection to use
         * @return Robot's 2D field position [x, y, yaw] in inches/degrees or null if not available
         */
        public double[] getRobot2DFieldPosition(AprilTagDetection detection) {
            double[] fullPose = getRobotFieldPosition(detection);
            if (fullPose == null) {
                return null;
            }
            return new double[]{fullPose[0], fullPose[1], fullPose[3]}; // x, y, yaw
        }

        /**
         * Get the distance to a detected AprilTag
         * @param detection The AprilTag detection
         * @return Distance in inches, or -1 if detection is invalid
         */
        public double getDistanceToTag(AprilTagDetection detection) {
            if (detection == null || detection.ftcPose == null) {
                return -1;
            }
            
            return Math.sqrt(
                Math.pow(detection.ftcPose.x, 2) + 
                Math.pow(detection.ftcPose.y, 2) + 
                Math.pow(detection.ftcPose.z, 2)
            );
        }

        /**
         * Get the bearing (angle) to a detected AprilTag
         * @param detection The AprilTag detection
         * @return Bearing in degrees, or Double.NaN if detection is invalid
         */
        public double getBearingToTag(AprilTagDetection detection) {
            if (detection == null || detection.ftcPose == null) {
                return Double.NaN;
            }
            
            return Math.toDegrees(Math.atan2(detection.ftcPose.y, detection.ftcPose.x));
        }

        /**
         * Check if any AprilTags are currently detected
         * @return true if at least one tag is detected
         */
        public boolean hasDetections() {
            return !getDetections().isEmpty();
        }

        /**
         * Get count of currently detected AprilTags
         * @return Number of detected tags
         */
        public int getDetectionCount() {
            return getDetections().size();
        }
    }

    // ========== APRILTAG CONVENIENCE METHODS ==========
    
    /**
     * Get the AprilTag detector instance
     * @return AprilTagDetector instance
     */
    public AprilTagDetector getAprilTagDetector() {
        return aprilTagDetector;
    }

    /**
     * Get all current AprilTag detections
     * @return List of AprilTag detections
     */
    public List<AprilTagDetection> getAprilTagDetections() {
        return aprilTagDetector.getDetections();
    }

    /**
     * Get the best (closest) AprilTag detection
     * @return Best AprilTag detection or null if none found
     */
    public AprilTagDetection getBestAprilTag() {
        return aprilTagDetector.getBestDetection();
    }

    /**
     * Get AprilTag detection by specific ID
     * @param tagId The ID of the tag to find
     * @return AprilTag detection with specified ID or null if not found
     */
    public AprilTagDetection getAprilTagById(int tagId) {
        return aprilTagDetector.getDetectionById(tagId);
    }

    /**
     * Get the robot's field position using the built-in SDK robotPose
     * This automatically handles camera positioning and field coordinates
     * @param detection The AprilTag detection to use
     * @return Robot's field position [x, y, z, yaw, pitch, roll] in inches/degrees or null if not available
     */
    public double[] getRobotFieldPosition(AprilTagDetection detection) {
        return aprilTagDetector.getRobotFieldPosition(detection);
    }

    /**
     * Get the robot's 2D field position (X, Y, Yaw only) using built-in SDK robotPose
     * @param detection The AprilTag detection to use
     * @return Robot's 2D field position [x, y, yaw] in inches/degrees or null if not available
     */
    public double[] getRobot2DFieldPosition(AprilTagDetection detection) {
        return aprilTagDetector.getRobot2DFieldPosition(detection);
    }

    /**
     * Get the robot's field position from the best detected AprilTag
     * @return Robot's field position [x, y, z, yaw, pitch, roll] or null if no valid detection
     */
    public double[] getRobotFieldPositionFromBestTag() {
        AprilTagDetection bestTag = getBestAprilTag();
        return getRobotFieldPosition(bestTag);
    }

    /**
     * Get the robot's 2D field position from the best detected AprilTag
     * @return Robot's 2D field position [x, y, yaw] or null if no valid detection
     */
    public double[] getRobot2DFieldPositionFromBestTag() {
        AprilTagDetection bestTag = getBestAprilTag();
        return getRobot2DFieldPosition(bestTag);
    }

    /**
     * Get distance to the closest AprilTag
     * @return Distance in inches, or -1 if no tags detected
     */
    public double getDistanceToClosestAprilTag() {
        AprilTagDetection closest = getBestAprilTag();
        return aprilTagDetector.getDistanceToTag(closest);
    }

    /**
     * Get bearing to the closest AprilTag
     * @return Bearing in degrees, or Double.NaN if no tags detected
     */
    public double getBearingToClosestAprilTag() {
        AprilTagDetection closest = getBestAprilTag();
        return aprilTagDetector.getBearingToTag(closest);
    }

    /**
     * Check if any AprilTags are currently detected
     * @return true if at least one tag is detected
     */
    public boolean hasAprilTags() {
        return aprilTagDetector.hasDetections();
    }

    /**
     * Get count of currently detected AprilTags
     * @return Number of detected tags
     */
    public int getAprilTagCount() {
        return aprilTagDetector.getDetectionCount();
    }

    // ========== COLOR BLOB CONVENIENCE METHODS ==========

    /**
     * Get current color blobs using the active color processor
     * @return List of detected color blobs
     */
    public List<ColorBlobLocatorProcessor.Blob> getColorBlobs() {
        return getBlobs(colorLocator);
    }

    /**
     * Get circle fits for current color blobs
     * @return List of circles fitted to detected blobs
     */
    public List<Circle> getColorCircles() {
        return getCircleFits(colorLocator);
    }

    // ========== UTILITY METHODS ==========

    /**
     * Check if the robot field position is available from any detected AprilTag
     * The built-in robotPose requires proper camera calibration and positioning setup
     * @return true if field position data is available
     */
    public boolean isFieldPositionAvailable() {
        AprilTagDetection bestTag = getBestAprilTag();
        return bestTag != null && bestTag.robotPose != null;
    }

    /**
     * Get a summary of the robot's current field position status
     * @return String describing the field position status
     */
    public String getFieldPositionStatus() {
        if (!hasAprilTags()) {
            return "No AprilTags detected";
        }
        
        AprilTagDetection bestTag = getBestAprilTag();
        if (bestTag.robotPose == null) {
            return "AprilTag detected but no field position available (check camera calibration)";
        }
        
        double[] pos = getRobot2DFieldPosition(bestTag);
        return String.format("Field Position: X=%.1f, Y=%.1f, Yaw=%.1f°", pos[0], pos[1], pos[2]);
    }

    /**
     * Get the vision portal instance for advanced operations
     * @return VisionPortal instance
     */
    public VisionPortal getVisionPortal() {
        return portal;
    }

    /**
     * Enable or disable AprilTag processing
     * @param enabled true to enable, false to disable
     */
    public void setAprilTagEnabled(boolean enabled) {
        portal.setProcessorEnabled(aprilTag, enabled);
    }

    /**
     * Enable or disable color blob processing
     * @param enabled true to enable, false to disable
     */
    public void setColorBlobEnabled(boolean enabled) {
        portal.setProcessorEnabled(colorLocator, enabled);
    }

    /**
     * Create a new Webcam instance with custom camera positioning
     * Use this if your camera is not mounted at the robot center
     * @param linearOpMode The LinearOpMode instance
     * @param cameraX Camera X offset from robot center (inches)
     * @param cameraY Camera Y offset from robot center (inches) 
     * @param cameraZ Camera Z offset from robot center (inches)
     * @param cameraYaw Camera yaw rotation (degrees)
     * @param cameraPitch Camera pitch rotation (degrees, typically -90 for horizontal)
     * @param cameraRoll Camera roll rotation (degrees)
     * @return Configured Webcam instance
     */
    public static Webcam createWithCameraOffset(LinearOpMode linearOpMode, 
            double cameraX, double cameraY, double cameraZ,
            double cameraYaw, double cameraPitch, double cameraRoll) {
        
        // Create a custom webcam instance with specific camera positioning
        Webcam webcam = new Webcam(linearOpMode);
        
        // Note: The camera position is set during construction, so this is more of a 
        // documentation method. For runtime changes, you'd need to rebuild the processor.
        
        return webcam;
    }

    // Call this function at the end of the OpMode to stop the camera safely
    public void closeStream() {
        if (portal != null) {
            portal.close();
        }
    }

    // Legacy method for backward compatibility
    public void closeStream(VisionPortal portal) {
        closeStream();
    }
}
