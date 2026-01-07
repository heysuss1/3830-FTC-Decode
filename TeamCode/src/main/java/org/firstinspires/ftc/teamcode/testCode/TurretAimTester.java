package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp(name = "Turret Tester")
public class TurretAimTester extends LinearOpMode {
    Robot robot;

    public void runOpMode(){

        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            robot.shooter.turretTask();
            robot.follower.update();
        }

    }

}
