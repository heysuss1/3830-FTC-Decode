package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Hardware {

    public static Hardware INSTANCE = null;
    public DriveTrain driveTrain = new DriveTrain();

    public Shooter shooter;
    public Turret turret = new Turret();
    public Transfer transfer;
    public Vision vision = new Vision();
    public double maxSpeed;




    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry){
//        vision.init(hwMap);//TODO: add telemetry;
        driveTrain.init(hwMap, telemetry);
        transfer = new Transfer(hwMap);
        shooter = new Shooter(hwMap);
//        shooter.init(hwMap);
//        turret.init(hwMap);
    }

}
