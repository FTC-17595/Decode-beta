package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ArtifactHandlingSystem {

    private final DcMotor outtakeMotor;
    private final DcMotor intakeMotor;
    private final DcMotor containerMotor;
//    private final DcMotor leftContainerMotor;
//    private final DcMotor rightContainerMotor;
    private final Servo flapServo;
    private final LinearOpMode linearOpMode;
    private double launchFactor;
    private long flapDownTime = 0;
    private boolean isMotorRecovering = false;
    private static final long MOTOR_RECOVERY_DELAY = 300;

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = linearOpMode.hardwareMap.dcMotor.get("outtakeMotor");
        this.intakeMotor = linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.containerMotor = linearOpMode.hardwareMap.dcMotor.get("containerMotor");
        this.flapServo = linearOpMode.hardwareMap.servo.get("flapServo");
        this.linearOpMode = linearOpMode;
//        this.leftContainerMotor = linearOpMode.hardwareMap.dcMotor.get("leftContainerMotor");
//        this.rightContainerMotor = linearOpMode.hardwareMap.dcMotor.get("rightContainerMotor");
    }

    public void configureMotorModes() {
        outtakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);
//        leftContainerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightContainerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        launchFactor = TeleOpConstants.SHORT_SHOOTING_FACTOR;
    }

    public void intakeSystem(boolean intakeArtifact, boolean rejectArtifact) {
        if (intakeArtifact) {
            intakeMotor.setPower(1);
            containerMotor.setPower(1);
        } else if (rejectArtifact) {
            intakeMotor.setPower(-1);
            containerMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
            containerMotor.setPower(0);
        }
    }

//    public void containerSystem(boolean sendArtifact) {
//        intakeMotor.setPower(0);
//
//        if (sendArtifact) {
//            containerMotor.setPower(1);
//        } else {
//            containerMotor.setPower(0);
//        }
//    }

    public void flapSystem(boolean flapUp) {
        if (flapUp) {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_UP);
        } else {
            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);
        }
    }

//    public void flapSystemWithRecovery(boolean flapUp) {
//        if (flapUp) {
//            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_UP);
//            isMotorRecovering = false;
//        } else {
//            flapServo.setPosition(TeleOpConstants.FLAP_SERVO_DOWN);
//            flapDownTime = System.currentTimeMillis();
//            isMotorRecovering = true;
//        }
//    }
//
//    public void shootingSystemWithBoost(float shootArtifact, float rejectArtifact) {
//        if (shootArtifact > 0) {
//            double effectivePower = shootArtifact * launchFactor;
//
//            // Apply temporary boost if motor is recovering from flap compression
//            if (isMotorRecovering) {
//                long timeSinceFlap = System.currentTimeMillis() - flapDownTime;
//                if (timeSinceFlap < MOTOR_RECOVERY_DELAY) {
//                    // Boost power by 15% during recovery period
//                    effectivePower *= 1.15;
//                } else {
//                    isMotorRecovering = false;
//                }
//            }
//
//            outtakeMotor.setPower(Math.min(effectivePower, 1.0)); // Cap at 1.0
//        } else if (rejectArtifact > 0) {
//            outtakeMotor.setPower(-rejectArtifact * launchFactor);
//            isMotorRecovering = false;
//        } else {
//            outtakeMotor.setPower(0);
//            isMotorRecovering = false;
//        }
//    }

    public void shootAutoArtifact(){
        Thread ArtifactShoot = new Thread(() -> {
            long endTime = System.currentTimeMillis() + 4000; // 4 seconds from now

            // Start the outtake motor for shooting
            shootingSystem(1,0);

            while (System.currentTimeMillis() < endTime) {

                containerMotor.setPower(1);
//                containerSystem(true, false);

            }



            shootingSystem(0,0);
            System.out.println("Thread finished its 4-second run!");
        });

        // Start the thread
        ArtifactShoot.start();
    } /* 😟😟😟 */

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setPower(shootArtifact * launchFactor);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setPower(-rejectArtifact * launchFactor);
        } else {
            outtakeMotor.setPower(0);
        }
    }

    public void adjustShootingFactor(boolean increase, boolean decrease) {
        if (increase) {
            launchFactor += 0.01;
        } else if (decrease) {
            launchFactor -= 0.01;
        }
    }

    public void switchShootingFactor(boolean switch_f) {
        if (switch_f) {
            launchFactor = launchFactor == TeleOpConstants.SHORT_SHOOTING_FACTOR ? TeleOpConstants.LONG_SHOOTING_FACTOR : TeleOpConstants.SHORT_SHOOTING_FACTOR;
        }
    }

    public void shootingSystemAuto(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0) {
            outtakeMotor.setPower(AutoConstants.AUTO_ARTIFACT_SHOOT_POWER);
        } else if (rejectArtifact > 0) {
            outtakeMotor.setPower(-rejectArtifact);
        } else {
            outtakeMotor.setPower(0);
        }
    }


    public void displayTelemetry() {
        linearOpMode.telemetry.addData("Outtake Motor Power", outtakeMotor.getPower());
        linearOpMode.telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        linearOpMode.telemetry.addData("Container Motor Power", containerMotor.getPower());
        linearOpMode.telemetry.addData("Flap Servo Position", flapServo.getPosition());
        linearOpMode.telemetry.addData("Shooting Factor", launchFactor);
//        linearOpMode.telemetry.addData("Left Container Power", leftContainerMotor.getPower());
//        linearOpMode.telemetry.addData("Right Container Power", rightContainerMotor.getPower());
    }
}
