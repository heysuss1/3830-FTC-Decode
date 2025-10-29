package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Hardware {

    public static Hardware INSTANCE = null;
    public DriveTrain driveTrain = new DriveTrain();

    public Shooter shooter = new Shooter();
    public Intake intake = new Intake();
    public Turret turret = new Turret();
    public Vision vision = new Vision();
    public double maxSpeed;




    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry){
        vision.init(hwMap);//TODO: add telemetry;
        driveTrain.init(hwMap, telemetry);
        intake.init(hwMap); //TODO: add telemetry
        shooter.init(hwMap);
        turret.init(hwMap);
    }

}
