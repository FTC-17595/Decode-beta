package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

public class Webcam {
    private final String webcamName = "Webcam 1"; // MODIFY THIS BASED ON WEBCAM NAME

    private final LinearOpMode linearOpMode;
    private final RobotControls robotControls;
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal; // Don't make this final

    public Webcam(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.robotControls = new RobotControls(linearOpMode);
        this.colorLocator = createProcessor(false); // Initialize with a default color
        this.portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
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

    // Call this function at the end of the OpMode to stop the camera safely
    public void closeStream(VisionPortal portal) {
        if (portal != null) {
            portal.close();
        }
    }
}
