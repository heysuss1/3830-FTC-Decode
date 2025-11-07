package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {


    // 0 to 3.3 volts crservo

    public CRServo intakeServo;

    public void init(HardwareMap hwMap){
        intakeServo = hwMap.get(CRServo.class, "intakeServo");
    }

    public void startIntake(){
        intakeServo.setPower(1);
    }
    public void stopIntake(){
        intakeServo.setPower(0);
    }
}
