package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class  Vision {

    Limelight3A limelight;


    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }



    public RobotConstants.Motif getMotif(){
        LLResult result = limelight.getLatestResult();


        int id;
        RobotConstants.Motif motif = RobotConstants.Motif.NULL;
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()){
            id = result.getFiducialResults().get(0).getFiducialId();
        } else {
            id = -1;
        }
        switch (id){
            case 21:
                motif = RobotConstants.Motif.GPP;
                break;
            case 22:
                motif = RobotConstants.Motif.PGP;
                break;
            case 23:
                motif = RobotConstants.Motif.PPG;
                break;
        }
        return motif;

    }

    //TODO:
}
