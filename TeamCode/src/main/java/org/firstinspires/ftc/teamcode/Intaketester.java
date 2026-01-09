package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp (name = "intake motor tester")
public class Intaketester extends LinearOpMode {
    TestHardware robot;

    public void runOpMode(){
        robot = new TestHardware(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            robot.setIntakeUptakePower(1,1);
        }
    }
}
