package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MechanismSubsystem {

    private DcMotorEx outtakeMotor;
    private DcMotor intakeMotor;
    private DcMotor containerMotor;
    private Servo flapServo;

    public static final double FLAP_UP = 0.5; 
    public static final double FLAP_DOWN = 0.0; 
    public static final double SHOOT_POWER = 0.72;

    public MechanismSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        containerMotor = hardwareMap.get(DcMotor.class, "containerMotor");
        flapServo = hardwareMap.get(Servo.class, "flapServo");

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flapServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
        containerMotor.setPower(power);
    }

    public void startShooter() {
        outtakeMotor.setPower(SHOOT_POWER);
    }

    public void stopShooter() {
        outtakeMotor.setPower(0);
    }

    public void setFlap(boolean up) {
        if (up) flapServo.setPosition(FLAP_UP);
        else    flapServo.setPosition(FLAP_DOWN);
    }
    
    public void shootArtifact() {
        setFlap(true);
    }
    
    public void resetFlap() {
        setFlap(false);
    }
}
