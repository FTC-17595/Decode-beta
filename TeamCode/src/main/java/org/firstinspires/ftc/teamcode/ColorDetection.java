package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class ColorDetection {
    private final ColorSensor frontLeftColorSensor;
    private final ColorSensor frontRightColorSensor;
    private final ColorSensor middleColorSensor;
    private final ColorSensor backColorSensor;
    private final Servo rgbIndicator;
    private final Servo outtakeIndicator;
    private boolean celebrateOn = false;
    private boolean previousCelebrate = false;
    private int g_artifactCount = 0;
    private final LinearOpMode linearOpMode;
    private final FtcDashboard dashboard;

    public ColorDetection(LinearOpMode linearOpMode) {
        frontLeftColorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "frontLeftColorSensor");
        frontRightColorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "frontRightColorSensor");
        middleColorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "middleColorSensor");
        backColorSensor = linearOpMode.hardwareMap.get(ColorSensor.class, "backColorSensor");
        rgbIndicator = linearOpMode.hardwareMap.get(Servo.class, "artifactIndicator");
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

        } else if (wasCelebrateOn) {
            // Clear both indicators when celebration is turned off
            rgbIndicator.setPosition(TeleOpConstants.BLANK);
            outtakeIndicator.setPosition(TeleOpConstants.BLANK);
        }
    }

    private float[] getHSV(ColorSensor colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    private int detectArtifact(ColorSensor colorSensor) {
        float[] hsv = getHSV(colorSensor);
        float hue   = hsv[0];
        float value = hsv[2];

        if (value < 0.5) { // ignore dark / nothing
            return 0;
        }

        // Check hue ranges
        if (hue > 140 && hue < 180) {
            return 1;
        } else if (hue > 181 && hue < 260) {
            return 1;
        } else {
            return 0;
        }
    }


    public void setRGBIndicator() {
        // Don't update if celebrating
        if (celebrateOn) {
            return;
        }

        final int artifactCount =
                ((detectArtifact(frontLeftColorSensor) == 1 || detectArtifact(frontRightColorSensor) == 1) ? 1 : 0)
                + detectArtifact(middleColorSensor)
                + detectArtifact(backColorSensor);

        g_artifactCount = artifactCount;

        switch (artifactCount) {
            case 3:
                rgbIndicator.setPosition(TeleOpConstants.GREEN);
                break;

            case 2:
                rgbIndicator.setPosition(TeleOpConstants.ORANGE);
                break;

            case 1:
                rgbIndicator.setPosition(TeleOpConstants.RED);
                break;

            default:
                rgbIndicator.setPosition(TeleOpConstants.BLANK);
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

        boolean artifactInBack = detectArtifact(backColorSensor) == 1;
        boolean reachedTargetVelocity = Math.abs(actualVelocity - targetVelocity) <= 15;


        if (artifactInBack && reachedTargetVelocity) {
            outtakeIndicator.setPosition(TeleOpConstants.BLUE);
        } else if (artifactInBack ^ reachedTargetVelocity) {
            outtakeIndicator.setPosition(TeleOpConstants.VIOLET);
        } else {
            outtakeIndicator.setPosition(TeleOpConstants.BLANK);
        }
    }

    private double getRGBIndicatorPosition() {
        return rgbIndicator.getPosition();
    }

    private boolean isCelebrateOn() {
        return celebrateOn;
    }

    public void displayTelemetry() {
        int FLred = frontLeftColorSensor.red();
        int FLgreen = frontLeftColorSensor.green();
        int FLblue = frontLeftColorSensor.blue();

        int FRred = frontRightColorSensor.red();
        int FRgreen = frontRightColorSensor.green();
        int FRblue = frontRightColorSensor.blue();

        int Mred = middleColorSensor.red();
        int Mgreen = middleColorSensor.green();
        int Mblue = middleColorSensor.blue();

        int Bred = backColorSensor.red();
        int Bgreen = backColorSensor.green();
        int Bblue = backColorSensor.blue();

        float[] FLhsv = getHSV(frontLeftColorSensor);
        float[] FRhsv = getHSV(frontRightColorSensor);
        float[] Mhsv = getHSV(middleColorSensor);
        float[] Bhsv = getHSV(backColorSensor);

        linearOpMode.telemetry.addData("Detected Color FL", detectArtifact(frontLeftColorSensor));
        linearOpMode.telemetry.addData("Detected Color FR", detectArtifact(frontRightColorSensor));
        linearOpMode.telemetry.addData("Detected Color M", detectArtifact(middleColorSensor));
        linearOpMode.telemetry.addData("Detected Color B", detectArtifact(backColorSensor));
        linearOpMode.telemetry.addData("FLred", FLred);
        linearOpMode.telemetry.addData("FLgreen", FLgreen);
        linearOpMode.telemetry.addData("FLblue", FLblue);
        linearOpMode.telemetry.addData("FRred", FRred);
        linearOpMode.telemetry.addData("FRgreen", FRgreen);
        linearOpMode.telemetry.addData("FRblue", FRblue);
        linearOpMode.telemetry.addData("FLhue", FLhsv[0]);
        linearOpMode.telemetry.addData("FLsaturation", FLhsv[1]);
        linearOpMode.telemetry.addData("FLvalue", FLhsv[2]);
        linearOpMode.telemetry.addData("FRhue", FRhsv[0]);
        linearOpMode.telemetry.addData("FRsaturation", FRhsv[1]);
        linearOpMode.telemetry.addData("FRvalue", FRhsv[2]);
        linearOpMode.telemetry.addData("Mhue", Mhsv[0]);
        linearOpMode.telemetry.addData("Msaturation", Mhsv[1]);
        linearOpMode.telemetry.addData("Mvalue", Mhsv[2]);
        linearOpMode.telemetry.addData("Bhue", Bhsv[0]);
        linearOpMode.telemetry.addData("Bsaturation", Bhsv[1]);
        linearOpMode.telemetry.addData("Bvalue", Bhsv[2]);
        linearOpMode.telemetry.addData("Mred", Mred);
        linearOpMode.telemetry.addData("Mgreen", Mgreen);
        linearOpMode.telemetry.addData("Mblue", Mblue);
        linearOpMode.telemetry.addData("Bred", Bred);
        linearOpMode.telemetry.addData("Bgreen", Bgreen);
        linearOpMode.telemetry.addData("Bblue", Bblue);
        linearOpMode.telemetry.addData("Amount of Artifacts in Robot", g_artifactCount);
        linearOpMode.telemetry.addData("RGB Position", getRGBIndicatorPosition());
        linearOpMode.telemetry.addData("Celebrate Mode", isCelebrateOn());

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("detectedColorFL", detectArtifact(frontLeftColorSensor));
        packet.put("detectedColorFR", detectArtifact(frontRightColorSensor));
        packet.put("detectedColorM", detectArtifact(middleColorSensor));
        packet.put("detectedColorB", detectArtifact(backColorSensor));
        packet.put("g_artifactCount", g_artifactCount);
        packet.put("rgbPosition", getRGBIndicatorPosition());
        packet.put("celebrateMode", isCelebrateOn());
        dashboard.sendTelemetryPacket(packet);
    }
}