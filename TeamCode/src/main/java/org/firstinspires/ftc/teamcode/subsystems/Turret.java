package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    // hooray

    public void init(HardwareMap hwMap){
        leftMotor = hwMap.get(DcMotorEx.class, "leftTurretMotor");
        rightMotor = hwMap.get(DcMotorEx.class, "rightTurretMotor");
    }

}
