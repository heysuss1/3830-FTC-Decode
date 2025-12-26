package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer{

    public static final double INTAKE_POWER = 1.0;
    public static final double FEED_POWER = 1;
    public enum Transfer_state {
        OFF,
        MOVING_FORWARD,
        MOVING_BACKWARD,
        WAITING_TO_SHOOT,
        SHOOTING,

    }

    public DcMotorEx intakeMotor;
    public DcMotorEx feedMotor;



    public Transfer(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        feedMotor = hwMap.get(DcMotorEx.class, "feedMotor");
        feedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        feedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        feedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedMotor.setPower(0);
    }

    public void startIntake(){
        intakeMotor.setPower(INTAKE_POWER);
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
    }

    public void stopFeed(){
        feedMotor.setPower(0);
    }
    public void setFeedIntakeMode(boolean hasBall){
        if (hasBall){
            feedMotor.setPower(0);
        } else {
            feedMotor.setPower(0.3);
        }

    }
    public void setIntakeMode(boolean hasBall){
        setFeedIntakeMode(hasBall);
        startIntake();
    }

    public void stopTransfer(){
        stopFeed();
        stopIntake();
    }


    public void setOuttakeMode(){
        feedMotor.setPower(-1);
        intakeMotor.setPower(-1);
    }
    public void startFeed(){
        feedMotor.setPower(FEED_POWER);
    }
    public void setFeedMode(){
        startIntake();
        startFeed();
    }
}
