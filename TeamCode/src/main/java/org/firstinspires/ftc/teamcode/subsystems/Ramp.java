package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Ramp {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret

    public DcMotorEx rampMotor;

    // hooray

    public void init(HardwareMap hwMap){
        rampMotor = hwMap.get(DcMotorEx.class, "rampMotor");
    }

    public void startRamp(){
        rampMotor.setPower(.67);
    }
    public void stopRamp(){
        rampMotor.setPower(0);
    }





}