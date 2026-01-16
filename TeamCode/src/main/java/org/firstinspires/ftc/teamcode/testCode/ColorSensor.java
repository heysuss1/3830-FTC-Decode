package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;



@TeleOp (name = "Color sensor tester")
public class ColorSensor extends LinearOpMode {

    Robot robot;
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Number of BALLS", robot.intakeUptake.getNumberOfBallsStored());
            telemetry.addData("Is uptake empty", robot.intakeUptake.isUptakeEmpty());
            telemetry.addData("Color Sensor 1 On", robot.intakeUptake.hasBall1());
            telemetry.addData("Color Sensor 1 Distance", robot.intakeUptake.getColorSensor1Distance());

            telemetry.addData("Color Sensor 2 On", robot.intakeUptake.hasBall2());
            telemetry.addData("Color Sensor 2 Distance", robot.intakeUptake.getColorSensor2Distance());

            telemetry.addData("Color Sensor 3 On", robot.intakeUptake.hasBall3());
            telemetry.addData("Color Sensor 3 Distance", robot.intakeUptake.getColorSensor3Distance());
            telemetry.update();
        }
    }
}
