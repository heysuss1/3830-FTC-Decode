package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret

    public DcMotorEx shootingMotor;
    public Servo pitchServo;
    public DcMotorEx turretRotation;

    // hooray

    public void init(HardwareMap hwMap){
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        shootingMotor = hwMap.get(DcMotorEx.class, "shootingMotor");
        turretRotation = hwMap.get(DcMotorEx.class, "turretMotor");
    }

    public double getCannonVelocity(){
       return shootingMotor.getVelocity();
    }

}

