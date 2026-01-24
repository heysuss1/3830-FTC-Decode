package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@TeleOp (name = "LED Test")
public class LEDTest extends LinearOpMode {
    Robot robot;





    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.circle){
                robot.intakeUptake.greenLED.on();
                robot.intakeUptake.redLED.off();
            }
            if (gamepad1.cross){
                robot.intakeUptake.greenLED.on();
                robot.intakeUptake.redLED.on();
            } if (gamepad1.square){
                robot.intakeUptake.greenLED.off();
                robot.intakeUptake.redLED.off();
            }
            if (gamepad1.triangle){
                    robot.intakeUptake.greenLED.off();
                    robot.intakeUptake.redLED.on();
            }
        }

    }

}
