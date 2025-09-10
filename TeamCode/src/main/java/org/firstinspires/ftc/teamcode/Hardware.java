package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class Hardware {

    public static Hardware INSTANCE = null;
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Vision vision = new Vision();

    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }

    public void init(HardwareMap hwMap){
        vision.init(hwMap);
        driveTrain.init(hwMap);
        intake.init(hwMap);
    }

}
