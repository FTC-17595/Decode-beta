package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class SimpleAprilTagHelper {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection lastDetection;

    public SimpleAprilTagHelper(HardwareMap hardwareMap, String webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(
                        (int) TeleOpConstants.OPENCV_IMAGE_WIDTH,
                        (int) TeleOpConstants.OPENCV_IMAGE_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

    }

    /** Call this each loop to update detection */
    public void updateDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        lastDetection = null;
        for (AprilTagDetection detection : detections) {
/*            if (detection.id == 21 || detection.id == 22 || detection.id == 23) {
                lastDetection = detection;
                break;

*/              if (detection.id == 21) {
                    lastDetection = detection;
                    break;
                }
            }
        }


    /** Returns 21, 22, or 23 if detected; otherwise -1 */
    public int getTagId() {
        return lastDetection != null ? lastDetection.id : -1;
    }

    /** Release camera resources */
    public void close() {
        visionPortal.close();
    }
}
