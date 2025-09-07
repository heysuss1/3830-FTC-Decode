package org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    public DcMotorEx lf;
    public DcMotorEx rf;
    public DcMotorEx lb;

    public DcMotorEx rb;



    public DriveTrain(){

    }
    public void initialize(){
        lf = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "lf");
        rf = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "rf");
        lb = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "lb");
        rb = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "rb");
    }
}
