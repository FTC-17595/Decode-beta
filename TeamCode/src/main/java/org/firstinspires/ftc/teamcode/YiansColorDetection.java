package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class YiansColorDetection {
    private final ColorSensor colorSensor;

    private final LinearOpMode linearOpMode;

    public YiansColorDetection(LinearOpMode linearOpMode) {
        colorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        this.linearOpMode = linearOpMode;
    }

    public float[] getHSV(ColorSensor colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    public String detectColor(ColorSensor colorSensor) {
        float[] hsv = getHSV(colorSensor);
        float hue   = hsv[0];
        float value = hsv[2];

        if (value < 0.5) { // ignore dark / nothing
            return "Unknown";
        }

        // Check hue ranges
        if (hue > 140 && hue < 180) {
            return "Green";
        } else if (hue > 200 && hue < 260) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }
}
