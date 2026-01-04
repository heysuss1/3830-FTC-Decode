package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;

public class  Vision {

    Limelight3A limelight;


    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }



    public Robot.Motif getMotif(){
        LLResult result = limelight.getLatestResult();


        int id;
        Robot.Motif motif = Robot.Motif.NULL;
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()){
            id = result.getFiducialResults().get(0).getFiducialId();
        } else {
            id = -1;
        }
        switch (id){
            case 21:
                motif = Robot.Motif.GPP;
                break;
            case 22:
                motif = Robot.Motif.PGP;
                break;
            case 23:
                motif = Robot.Motif.PPG;
                break;
        }
        return motif;

    }

    //TODO:
}
