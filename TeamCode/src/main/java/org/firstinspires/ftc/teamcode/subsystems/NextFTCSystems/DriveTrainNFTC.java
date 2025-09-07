package org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

public class DriveTrainNFTC extends Subsystem {
    public static final DriveTrainNFTC INSTANCE = new DriveTrainNFTC();
    public DcMotorEx lf;
    public DcMotorEx rf;
    public DcMotorEx lb;

    public DcMotorEx rb;



    public DriveTrainNFTC(){

    }
    public void initialize(){
        lf = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "lf");
        rf = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "rf");
        lb = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "lb");
        rb = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "rb");
    }
}
