package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp

public class IntakeTest extends LinearOpMode{
    private final DcMotor intakeMotor;
    private final DcMotor containerMotor;
    private final Servo flapServo;
    private final LinearOpMode linearOpMode;
//    private final DcMotorEx outtakeMotor;


        private ArtifactHandlingSystem artifactHandlingSystem;
        private RobotControls robotControls;
        private DriveTrain driveTrain;
        private ColorDetection colorDetection;

    private double intakePower;


    public IntakeTest(LinearOpMode linearOpMode) {
//            this.outtakeMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.dcMotor.get("containerMotor");
        this.flapServo = linearOpMode.hardwareMap.servo.get("flapServo");
        this.linearOpMode = linearOpMode;
    }




    @Override
    public void runOpMode() throws InterruptedException {


        configureMotorModes();

        waitForStart();
        if (isStopRequested()) return;
        mainTeleOpLoop();
    }


        private void mainTeleOpLoop() throws InterruptedException {
            while (opModeIsActive()) {
                boolean accept =gamepad1.right_bumper;
                boolean reject = gamepad1.left_bumper;
                float intake = gamepad1.right_trigger;
                adjustIntakePower(accept,reject);
                intakeSystem(intake);
                displayTelemetry();
            }
        }

    public void intakeSystem(float intake) {
        if (intake > 0) {
            intakeMotor.setPower(1);
        containerMotor.setPower(intakePower);
        } else {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
        }
    }
        private void displayTelemetry() {
            telemetry.addData("Intake Motor Power", intakeMotor.getPower());
            telemetry.update();
        }

    public void configureMotorModes() {


        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);

        intakePower = TeleOpConstants.SHORT_RANGE_VELOCITY;
    }

    public void adjustIntakePower(boolean increase, boolean decrease) {
    if (increase) {
        intakePower += 0.25;
    } else if (decrease) {
        intakePower -= 0.25;
    }
    // Clamp velocity to safe operating limits
    // Max theoretical = 11,200 ticks/sec;
        intakePower = Math.max(0, Math.min(intakePower, 1));
}
}


// Define target velocities in ticks per second
        // RS-555 Motor Specs:
        // - 6000 RPM no-load speed
        // - 28 PPR encoder with quadrature (x4) = 112 CPR (counts per revolution)
        // - Gear ratio: 1:1 (direct drive)
        //
        // Max theoretical velocity calculation:
        // 6000 RPM ÷ 60 = 100 rev/sec
        // 100 rev/sec × 112 ticks/rev = 11,200 ticks/sec
        //
        // Recommended operating range: 70-85% of max for PID stability and load handling




