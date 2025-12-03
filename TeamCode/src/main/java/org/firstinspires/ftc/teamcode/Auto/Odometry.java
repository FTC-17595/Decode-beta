/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

public class Odometry {

    GoBildaPinpointDriver odo;
    double oldTime = 0;
    double frequency = 0;
    LinearOpMode linearOpMode;

    public Odometry(LinearOpMode linearOpMode) {
        this.odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        this.linearOpMode = linearOpMode;
    }

    public void configureOdo() {

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderResolution(13.26291192, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.recalibrateIMU();
        odo.resetPosAndIMU();

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        linearOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        linearOpMode.telemetry.addData("Heading Scalar", odo.getYawScalar());
        linearOpMode.telemetry.update();

        linearOpMode.resetRuntime();
    }

    public void updateOdo() {
        odo.update();
        //odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        if (linearOpMode.gamepad1.a) {
            odo.resetPosAndIMU();
        }

        if (linearOpMode.gamepad1.b) {
            odo.recalibrateIMU();
        }

        double newTime = linearOpMode.getRuntime();
        double loopTime = newTime - oldTime;
        double frequency = 1 / loopTime;
        oldTime = newTime;
        this.frequency = frequency;
    }

    public void displayTelemetry() {
        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Position", data);
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.MM), odo.getVelY(DistanceUnit.MM), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
        linearOpMode.telemetry.addData("Velocity", velocity);
        linearOpMode.telemetry.addData("Status", odo.getDeviceStatus());
        linearOpMode.telemetry.addData("Pinpoint Frequency", odo.getFrequency());
        linearOpMode.telemetry.addData("REV Hub Frequency: ", frequency);
        linearOpMode.telemetry.update();
    }
}