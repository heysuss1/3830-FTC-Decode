package org.firstinspires.ftc.teamcode.autos;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous (name = "Red Side 2")
public class BlueSideAuto2 extends LinearOpMode {
    Timer pathTimer;
    Hardware robot = Hardware.getInstance();
    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        waitForStart();
        pathTimer = new Timer();
        robot.driveTrain.setPower(0.5, 0.5, -0.5, -0.5);
        while (pathTimer.getElapsedTimeSeconds() < 3){

        }

    }
}
