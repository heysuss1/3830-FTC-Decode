package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class ArcSortingBot {

    public Shooter shooter;
    public Turret turret = new Turret();

    public static ArcSortingBot INSTANCE = null;

    public void init(HardwareMap hwMap){
        shooter = new Shooter(hwMap);
        turret.init(hwMap);
    }

    public static ArcSortingBot getInstance(){
        if (INSTANCE == null){
            INSTANCE = new ArcSortingBot();
        }
        return INSTANCE;
    }
}