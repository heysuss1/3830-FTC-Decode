package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ArcSortingBot {

    public Shooter shooter;

    public DcMotorEx intakeMotor;
    public GoBildaPinpointDriver pinpoint;
    public static ArcSortingBot INSTANCE = null;

    public void init(HardwareMap hwMap){
        shooter = new Shooter(hwMap);
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static ArcSortingBot getInstance(){
        if (INSTANCE == null){
            INSTANCE = new ArcSortingBot();
        }
        return INSTANCE;
    }
}