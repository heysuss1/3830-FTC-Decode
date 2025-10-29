package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public DcMotorEx intakeMotor;

    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
    }



    public void startIntake(){
        intakeMotor.setPower(0.67);
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
    }
}
