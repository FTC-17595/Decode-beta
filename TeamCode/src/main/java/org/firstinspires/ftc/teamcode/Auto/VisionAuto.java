package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class VisionAuto {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    public VisionAuto(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .build();
    }


    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void close() {
        visionPortal.close();
    }
}
