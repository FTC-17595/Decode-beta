package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Auto Tune PID", group = "Tuning")
public class AutoTuneDrivetrain extends LinearOpMode {
    // ChatGPT did the most of this, I just added all the odometry stuff

    // Mecanum motors
    private DcMotorEx lf, lr, rf, rr; // Also, I had to speedrun this, so we have motors like "left forward" and "right rear"

    // Pinpoint odometry
    private GoBildaPinpointDriver pinpoint;

    // Dashboard
    private FtcDashboard dashboard;

    // Tuned values
    public static double kS = 0;
    public static double kP = 0, kI = 0, kD = 0;
    public static double kPH = 0, kIH = 0, kDH = 0;

    // Translation and heading targets
    public static double TARGET_DIST_MM = 600;
    public static double TARGET_HEADING_DEG = 90;

    @Override
    public void runOpMode() throws InterruptedException {

        // -------------------------
        // Hardware
        // -------------------------
        lf = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        lr = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        rf = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rr = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        for (DcMotorEx m : new DcMotorEx[]{lf, lr, rf, rr}) {
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("Auto PID Tuner is ready");
        telemetry.addLine("Press START to begin.");
        telemetry.update();
        waitForStart();

        // -------------------------
        // Step 1: Tune static friction
        // -------------------------
        kS = tuneKS();

        telemetry.addData("kS", kS);
        telemetry.update();
        sleep(1500);

        // -------------------------
        // Step 2: Translation PID
        // -------------------------
        double[] translationPID = tuneTranslationPID(kS);
        kP = translationPID[0];
        kI = translationPID[1];
        kD = translationPID[2];

        // -------------------------
        // Step 3: Heading PID
        // -------------------------
        double[] headingPID = tuneHeadingPID();
        kPH = headingPID[0];
        kIH = headingPID[1];
        kDH = headingPID[2];

        // -------------------------
        // Done
        // -------------------------
        telemetry.addLine("Tuning Complete!");
        telemetry.addData("kS", kS);
        telemetry.addData("kP/kI/kD", kP + "/" + kI + "/" + kD);
        telemetry.addData("kPH/kIH/kDH", kPH + "/" + kIH + "/" + kDH);
        telemetry.update();

        while (opModeIsActive()) idle();
    }

    // -------------------------
    // Static friction
    // -------------------------
    private double tuneKS() {
        telemetry.addLine("Tuning kS...");
        telemetry.update();

        double power = 0;
        final double step = 0.0025;

        pinpoint.resetPosAndIMU();
        sleep(200);

        while (opModeIsActive()) {
            power += step;
            setAll(power);

            double velY = pinpoint.getVelY(DistanceUnit.MM); // mm/s

            telemetry.addData("Power", power);
            telemetry.addData("Velocity mm/s", velY);
            telemetry.update();

            if (Math.abs(velY) > 5) {
                setAll(0);
                return power;
            }

            sleep(40);
        }
        return 0.05;
    }

    // -------------------------
    // Translation PID with graphing
    // -------------------------
    private double[] tuneTranslationPID(double kS) {

        telemetry.addLine("Tuning Translation PID...");
        telemetry.update();

        double Ku = 0;
        double Pu = 0;

        for (double testP = 0.003; testP < 2.0 && opModeIsActive(); testP += 0.003) {

            pinpoint.resetPosAndIMU();
            sleep(200);

            List<Double> errorLog = new ArrayList<>();
            List<Double> timeLog = new ArrayList<>();

            long startTime = System.nanoTime();

            while ((System.nanoTime() - startTime) < 1_000_000_000L) {

                double pos = pinpoint.getPosY();
                double error = TARGET_DIST_MM - pos;

                double output = testP * error;
                if (output != 0) output += Math.signum(output) * kS;

                setAll(output);

                double t = (System.nanoTime() - startTime) / 1e9;
                errorLog.add(error);
                timeLog.add(t);

                // Send to dashboard graph
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Translation Error mm", error);
                packet.put("Translation Output", output);
                dashboard.sendTelemetryPacket(packet);

                sleep(20);
            }

            setAll(0);

            if (detectOscillation(errorLog)) {
                Ku = testP;
                Pu = estimatePeriod(errorLog, timeLog);
                break;
            }
        }

        double finalP = 0.6 * Ku;
        double finalI = (1.2 * Ku) / Pu;
        double finalD = (3 * Ku * Pu) / 40;

        return new double[]{finalP, finalI, finalD};
    }

    // -------------------------
    // Heading PID with graphing
    // -------------------------
    private double[] tuneHeadingPID() {

        telemetry.addLine("Tuning Heading PID...");
        telemetry.update();

        double Ku = 0;
        double Pu = 0;

        for (double testP = 0.003; testP < 2.0 && opModeIsActive(); testP += 0.003) {

            pinpoint.resetPosAndIMU();
            sleep(200);

            List<Double> errorLog = new ArrayList<>();
            List<Double> timeLog = new ArrayList<>();

            long startTime = System.nanoTime();

            while ((System.nanoTime() - startTime) < 1_000_000_000L) {

                double headingDeg = Math.toDegrees(pinpoint.getHeading());
                double error = TARGET_HEADING_DEG - headingDeg;
                error = ((error + 180) % 360) - 180;

                double output = testP * error;

                setHeadingPower(output);

                double t = (System.nanoTime() - startTime) / 1e9;
                errorLog.add(error);
                timeLog.add(t);

                // Dashboard graph
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Heading Error deg", error);
                packet.put("Heading Output", output);
                dashboard.sendTelemetryPacket(packet);

                sleep(20);
            }

            setAll(0);

            if (detectOscillation(errorLog)) {
                Ku = testP;
                Pu = estimatePeriod(errorLog, timeLog);
                break;
            }
        }

        double finalP = 0.6 * Ku;
        double finalI = (1.2 * Ku) / Pu;
        double finalD = (3 * Ku * Pu) / 40;

        return new double[]{finalP, finalI, finalD};
    }

    // -------------------------
    // Helpers (Oscillating Uschen?)
    // -------------------------
    private boolean detectOscillation(List<Double> e) {
        if (e.size() < 20) return false;
        double mean = e.stream().mapToDouble(x -> x).average().orElse(0);
        int zeroCross = 0;
        for (int i = 1; i < e.size(); i++)
            if (Math.signum(e.get(i)) != Math.signum(e.get(i - 1))) zeroCross++;
        return zeroCross > 6;
    }

    private double estimatePeriod(List<Double> e, List<Double> t) {
        double mean = e.stream().mapToDouble(x -> x).average().orElse(0);
        List<Double> crossings = new ArrayList<>();
        for (int i = 1; i < e.size(); i++)
            if ((e.get(i - 1) < mean && e.get(i) > mean) || (e.get(i - 1) > mean && e.get(i) < mean))
                crossings.add(t.get(i));
        if (crossings.size() < 4) return 0.5;
        return crossings.get(3) - crossings.get(1);
    }

    private void setAll(double p) {
        lf.setPower(p);
        lr.setPower(p);
        rf.setPower(p);
        rr.setPower(p);
    }

    private void setHeadingPower(double p) {
        lf.setPower(p);
        lr.setPower(p);
        rf.setPower(-p);
        rr.setPower(-p);
    }
}
