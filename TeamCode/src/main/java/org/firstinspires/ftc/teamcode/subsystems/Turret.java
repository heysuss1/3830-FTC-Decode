package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public Servo pitchServo;

    public DcMotorEx turretRotation;
    // hooray

    public void init(HardwareMap hwMap){
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        turretRotation = hwMap.get(DcMotorEx.class,"turretRotation");
    }

    public double ticksToDegrees(int ticks){
        double turretConversion = 6.7; //Change!!!
        return ticks * turretConversion;
    }

}
