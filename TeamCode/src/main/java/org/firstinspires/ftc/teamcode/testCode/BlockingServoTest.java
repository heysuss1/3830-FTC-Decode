package org.firstinspires.ftc.teamcode.testCode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp (name = "Blocking Servo Position Getter")
public class BlockingServoTest extends LinearOpMode {
    Robot robot;
    public static double blockingServo1Position;
    public static double blockingServo2Position;


    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            robot.intakeUptake.getBlockingServo1().setPosition(blockingServo1Position);
            robot.intakeUptake.getBlockingServo2().setPosition(blockingServo2Position);
        }
    }
}
