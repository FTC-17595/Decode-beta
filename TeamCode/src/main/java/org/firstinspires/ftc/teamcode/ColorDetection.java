package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Random;

public class ColorDetection {
    private final ColorSensor colorSensor;
    private final Servo rgbIndicator;
    private final Servo outtakeIndicator;
    private boolean celebrateOn = false;
    private boolean previousCelebrate = false;
    private final LinearOpMode linearOpMode;
    private final FtcDashboard dashboard;

    public ColorDetection(LinearOpMode linearOpMode) {
        colorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
        rgbIndicator = linearOpMode.hardwareMap.get(Servo.class, "rgbIndicator");
        outtakeIndicator = linearOpMode.hardwareMap.get(Servo.class, "outtakeIndicator");

        this.linearOpMode = linearOpMode;
        this.dashboard = FtcDashboard.getInstance();
    }

    public void celebrateToggle(boolean celebrate) throws InterruptedException {
        boolean wasCelebrateOn = celebrateOn;

        if (celebrate && !previousCelebrate) {
            celebrateOn = !celebrateOn;
        }

        previousCelebrate = celebrate;

        if (celebrateOn) {
            long currentTime = System.currentTimeMillis();
            double cyclePosition = (currentTime % TeleOpConstants.CELEBRATION_SPEED) / TeleOpConstants.CELEBRATION_SPEED;

            double triangleWave = cyclePosition < 0.5 ? cyclePosition * 2 : 2 - (cyclePosition * 2);

            double rainbowColor = TeleOpConstants.RED + (TeleOpConstants.VIOLET - TeleOpConstants.RED) * triangleWave;

            rgbIndicator.setPosition(rainbowColor);
            outtakeIndicator.setPosition(rainbowColor);

            Thread.sleep(250);
        } else if (wasCelebrateOn) {
            // Clear both indicators when celebration is turned off
            rgbIndicator.setPosition(TeleOpConstants.BLANK);
            outtakeIndicator.setPosition(TeleOpConstants.BLANK);
        }
    }

    private float[] getHSV() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    private String detectColor() {
        float[] hsv = getHSV();
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


    public void setRGBIndicator() {
        // Don't update if celebrating
        if (celebrateOn) {
            return;
        }

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

    public void setOuttakeIndicatorWithVelocity(double targetVelocity, double actualVelocity) {
        // Don't update if celebrating
        if (celebrateOn) {
            return;
        }

        // Check velocity status first (highest priority)
        if (targetVelocity == 0) {
            // Motor not being commanded - blank
            outtakeIndicator.setPosition(TeleOpConstants.BLANK);
            return;
        }

        if (Math.abs(actualVelocity - targetVelocity) <= 10) {
            // At target velocity - blue
            outtakeIndicator.setPosition(TeleOpConstants.BLUE);
            return;
        }

        // If not at target and motor is running:
        outtakeIndicator.setPosition(TeleOpConstants.BLANK);
    }

    private double getRGBIndicatorPosition() {
        return rgbIndicator.getPosition();
    }

    private boolean isCelebrateOn() {
        return celebrateOn;
    }

    public void displayTelemetry() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = getHSV();

        linearOpMode.telemetry.addData("Detected Color", detectColor());
        linearOpMode.telemetry.addData("Red", red);
        linearOpMode.telemetry.addData("Green", green);
        linearOpMode.telemetry.addData("Blue", blue);
        linearOpMode.telemetry.addData("Hue", hsv[0]);
        linearOpMode.telemetry.addData("Saturation", hsv[1]);
        linearOpMode.telemetry.addData("Value", hsv[2]);
        linearOpMode.telemetry.addData("RGB Position", getRGBIndicatorPosition());
        linearOpMode.telemetry.addData("Celebrate Mode", isCelebrateOn());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("detectedColor", detectColor());
        packet.put("red", red);
        packet.put("green", green);
        packet.put("blue", blue);
        packet.put("hue", hsv[0]);
        packet.put("saturation", hsv[1]);
        packet.put("value", hsv[2]);
        packet.put("rgbPosition", getRGBIndicatorPosition());
        packet.put("celebrateMode", isCelebrateOn());
        dashboard.sendTelemetryPacket(packet);
    }
}