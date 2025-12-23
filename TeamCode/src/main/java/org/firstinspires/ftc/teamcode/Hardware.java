package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Hardware {

    public static Hardware INSTANCE = null;
    public DriveTrain driveTrain = new DriveTrain();
    public Shooter shooter;
//    public Turret turret = new Turret();
    public Transfer transfer;
    private HardwareMap hwMap;
    private double voltage;
    public Vision vision = new Vision();
    public double maxSpeed;






    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }



    public void setVoltage(){
        voltage = hwMap.voltageSensor.iterator().next().getVoltage();
    }

    public double getVoltage(){
        return voltage;
    }
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.hwMap = hwMap;
        driveTrain.init(hwMap, telemetry);
        transfer = new Transfer(hwMap);
        shooter = new Shooter(hwMap);
//        turret.init(hwMap);
    }
}
