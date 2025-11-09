package org.firstinspires.ftc.teamcode.autos;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;

@Autonomous (name = "Red SIde")
public class QuickAuto extends OpMode {


    Timer pathTimer;
    Hardware robot = Hardware.getInstance();

    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    public void loop(){
        robot.driveTrain.setPower(0.5, 0.5, 0.5, 0.5);
    }
}
