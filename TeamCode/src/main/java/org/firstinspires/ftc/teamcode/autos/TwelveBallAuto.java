package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;


@Autonomous (name =  "12 Ball Auto")
public class TwelveBallAuto extends OpMode {


    //During this loop, using the DPAD, the team will be set
    public void init_loop(){
        RobotConstants.setTeam(RobotConstants.Team.BLUE);
    }


    public void init(){
    }

    public void buildRedPaths(){

    }
    public void buildBluePaths(){

    }

    public void loop(){

    }
}
