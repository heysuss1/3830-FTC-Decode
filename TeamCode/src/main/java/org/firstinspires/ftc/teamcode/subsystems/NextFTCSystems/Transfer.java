package org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

public class Transfer{

    public static final double INTAKE_POWER = 1.0;
    public static final double FEED_POWER = 0.7;
    public static final double RAMP_POWER = 1;
    public static final double RAMP_INTAKE_POWER = -0.2;



    public DcMotorEx rampMotor;
    public CRServo intakeServo;
    public DcMotorEx feedMotor;


    public Transfer(HardwareMap hwMap){
        intakeServo = hwMap.get(CRServo.class, "intakeServo");


        rampMotor = hwMap.get(DcMotorEx.class, "rampMotor");
        rampMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rampMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setPower(0);

        feedMotor = hwMap.get(DcMotorEx.class, "feedMotor");
        feedMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        feedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedMotor.setPower(0);
    }

    public void startIntake(){
        intakeServo.setPower(INTAKE_POWER);
    }
    public void stopIntake(){
        intakeServo.setPower(0);
    }
    public void stopRamp(){
        rampMotor.setPower(0);
    }

    public void setRampIntakeMode(){
        rampMotor.setPower(RAMP_INTAKE_POWER);
    }
    public void startRamp(){
        rampMotor.setPower(RAMP_POWER);
    }
    public void setIntakeMode(){
        startIntake();
        setRampIntakeMode();
    }
    public void startFeed(){
        feedMotor.setPower(FEED_POWER);
    }
    public void setFeedMode(){
        startIntake();
        startRamp();
        startFeed();
    }
}
