package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;

public class BlueSideAuto extends OpMode {

    public void init(){
       telemetry.addLine("Team Color: ");
       telemetry.update();
       if (gamepad1.dpad_left){
           telemetry.addData("Team Color: ", "blue");
           Constants.setTeam(Constants.Team.BLUE);
       }
        if (gamepad1.dpad_right){
            telemetry.addData("Team Color: ", "red");
            Constants.setTeam(Constants.Team.RED);
        }
    }
    public void loop(){

    }

}
