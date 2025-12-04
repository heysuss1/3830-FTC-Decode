package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer{

    public static final double INTAKE_POWER = 1.0;
    public static final double FEED_POWER = 1;
    public static final double RAMP_POWER = 1;
    public static final double RAMP_INTAKE_POWER = 0.65;

    public enum Transfer_state {
        OFF,
        MOVING_FORWARD,
        MOVING_BACKWARD,
        WAITING_TO_SHOOT,
        SHOOTING,

    }

    public DcMotorEx rampMotor;
    public DcMotorEx intakeMotor;
    public DcMotorEx feedMotor;


    public Transfer(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        rampMotor = hwMap.get(DcMotorEx.class, "rampMotor");
        rampMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rampMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rampMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampMotor.setPower(0);

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
    public void stopRamp(){
        rampMotor.setPower(0);
    }

    public void setRampIntakeMode(){
        rampMotor.setPower(RAMP_INTAKE_POWER);
    }
    public void startRamp(){
        rampMotor.setPower(RAMP_POWER);
    }
    public void stopFeed(){
        feedMotor.setPower(0);
    }
    public void setFeedIntakeMode(){feedMotor.setPower(-0.2);}
    public void setIntakeMode(){
        startIntake();
        setFeedIntakeMode();
        setRampIntakeMode();

    }

    public void setIntakeOff(){
        stopFeed();
        stopIntake();
        stopRamp();
    }


    public void moveBackwards(){
        rampMotor.setPower(-0.4);
        feedMotor.setPower(-0.6);
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
