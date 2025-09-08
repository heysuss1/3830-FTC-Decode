package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Hardware {
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();

    public void init(HardwareMap hwMap){
        driveTrain.init(hwMap);
        intake.init(hwMap);
    }

}
