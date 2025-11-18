package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class SampleSorter {
    private final ColorSensor colorSensor;
    private final ColorSensor colorSensor2;
    VisionAuto visionAuto;
    YiansColorDetection colorDetection;
    private final Servo containerServo;
    AprilTagDetection tag;
    int id;

    // This class is meant for the future in interleague.

    public SampleSorter(LinearOpMode linearOpMode) {
        colorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor2 = linearOpMode.hardwareMap.get(ColorSensor.class, "colorSensor2");
        containerServo = linearOpMode.hardwareMap.get(Servo.class, "sorterServo");
        visionAuto = new VisionAuto(linearOpMode.hardwareMap, "visionAuto");
        colorDetection = new YiansColorDetection(linearOpMode);

        // Replace "visionAuto.getDetections()" for something else later, like a variable that stores it.
        AprilTagDetection tag = visionAuto.getDetections().get(0);
        int id = tag.id;
    }

    public void sort() {
        // We are assuming the color sensor is, or will be, stationed so that it views the artifact that is to be dispensed first.
        // We are also assuming that our container servo, once time comes, will be compatible with this sorting code.
        // gpp, pgp, ppg
        String color1 = colorDetection.detectColor(colorSensor);
        String color2 = colorDetection.detectColor(colorSensor2);
        int detectedTag;
        if (color1.equals("Green")){
            detectedTag = 21;
        } else if (color2.equals("Green")) {
            detectedTag = 22;
        } else {
            detectedTag = 23;
        }

        // id-detectedTag represents the error in terms of ID, and we multiply that by 0.33 because shifting the servo by 0.33 will "shift" the detectedTag by 1, and of course we want detectedTag to equal the obelisk ID.
        double pos = containerServo.getPosition() + ((id-detectedTag) * 0.33);

        pos = pos % 1;

        containerServo.setPosition(pos);

    }


}


