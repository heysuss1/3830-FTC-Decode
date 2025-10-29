package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    Servo turretRotation;

    // hooray

    public void init(HardwareMap hwMap){
        turretRotation = hwMap.get(Servo.class, "turretRotation");
    }

}
