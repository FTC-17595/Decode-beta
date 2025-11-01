package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Random;

public class ColorDetection {
    private final ColorSensor colorSensor;
    private final Servo rgbIndicator;
    private boolean celebrateOn = false;
    private boolean previousCelebrate = false;
    private final LinearOpMode linearOpMode;

    public ColorDetection(LinearOpMode linearOpMode) {
        colorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        rgbIndicator = linearOpMode.hardwareMap.get(Servo.class, "rgbIndicator");

        this.linearOpMode = linearOpMode;
    }

    public void celebrateToggle(boolean celebrate) throws InterruptedException {
        if (celebrate && !previousCelebrate) {
            celebrateOn = !celebrateOn;
        }

        previousCelebrate = celebrate;

        if (celebrateOn) {
            Random random = new Random();
            double randomColor = TeleOpConstants.RED + (TeleOpConstants.VIOLET - TeleOpConstants.RED) * random.nextDouble();
            rgbIndicator.setPosition(randomColor);

            // Wait for 500 milliseconds
            Thread.sleep(250);
        }
    }

    private float hue() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv[0];
    }

    private String detectColor() {
        float hue = hue();

        if ((hue > 150) && (hue < 170)) {
            return "Green";
        } else if ((hue > 220) && (hue < 240)) {
            return "Purple";
        } else {
            return "Unknown";
        }
    }

    public void setRGBIndicator() {
        String detectedColor = detectColor();

        switch (detectedColor) {
            case "Green":
                rgbIndicator.setPosition(TeleOpConstants.GREEN);
                break;
            case "Purple":
                rgbIndicator.setPosition(TeleOpConstants.VIOLET);
                break;
            case "Unknown":
                rgbIndicator.setPosition(TeleOpConstants.BLANK);
                break;
        }
    }

    private double getRGBIndicatorPosition() {
        return rgbIndicator.getPosition();
    }

    private boolean isCelebrateOn() {
        return celebrateOn;
    }

    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Detected Color", detectColor());
        linearOpMode.telemetry.addData("Detected Hue", hue());
        linearOpMode.telemetry.addData("RGB Position", getRGBIndicatorPosition());
        linearOpMode.telemetry.addData("Celebrate Mode", isCelebrateOn());
    }
}
