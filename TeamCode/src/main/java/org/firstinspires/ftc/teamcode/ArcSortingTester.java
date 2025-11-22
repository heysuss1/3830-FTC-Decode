package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TestCode.ArcSortingBot;

public class ArcSortingTester extends LinearOpMode {

    RobotConstants.SystemState systemState;
    int arcShooterUpdate;
    ArcSortingBot ArcCy = ArcSortingBot.getInstance();
    public void runOpMode(){
        ArcCy.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
        }
    }
    public void updateRobotState(){
        int tolerance;
        double batteryVoltage;
        switch (arcShooterUpdate){
            case 0:


        }
    }

}

