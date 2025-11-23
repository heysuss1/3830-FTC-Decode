package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "Intake Motor Tester")
public class IntakeMotorTest extends LinearOpMode {
    ArcSortingBot robot = ArcSortingBot.getInstance();

    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            robot.intakeMotor.setPower(1);
        }
    }
}
