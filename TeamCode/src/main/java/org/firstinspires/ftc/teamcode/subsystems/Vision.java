package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

public class Vision {

    Limelight3A limelight;


    public void init(HardwareMap hwMap){
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }



    public Constants.Motif getMotif(){
        LLResult result = limelight.getLatestResult();


        int id;
        Constants.Motif motif = Constants.Motif.NULL;
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()){
            id = result.getFiducialResults().get(0).getFiducialId();
        } else {
            id = -1;
        }
        switch (id){
            case 21:
                motif = Constants.Motif.GPP;
                break;
            case 22:
                motif = Constants.Motif.PGP;
                break;
            case 23:
                motif = Constants.Motif.PPG;
                break;
        }
        return motif;

    }

    //TODO:
}
