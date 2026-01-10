package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp (name = "Shooter Direction test")
public class ShooterTester extends LinearOpMode {

    Robot robot;
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            if (gamepad1.square){
                robot.shooter.getTopShooterMotor().setPower(0.3);
                robot.shooter.getBottomShooterMotor().setPower(0.3);
            } else if (gamepad1.cross){
                robot.shooter.getTopShooterMotor().setPower(0.3);
                robot.shooter.getBottomShooterMotor().setPower(-0.3);
            } else if (gamepad1.circle){
                robot.shooter.getTopShooterMotor().setPower(-0.3);
                robot.shooter.getBottomShooterMotor().setPower(0.3);
            } else if (gamepad1.triangle){
                robot.shooter.getTopShooterMotor().setPower(-0.3);
                robot.shooter.getBottomShooterMotor().setPower(-0.3);
            } else{
                robot.shooter.getTopShooterMotor().setPower(0);
                robot.shooter.getBottomShooterMotor().setPower(0);
            }
        }


    }
}
