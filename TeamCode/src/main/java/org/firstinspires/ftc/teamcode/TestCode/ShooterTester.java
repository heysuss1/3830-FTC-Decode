package org.firstinspires.ftc.teamcode.TestCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.VelocityController;


@Config
@TeleOp (name = "Shooter Tester")
public class ShooterTester extends LinearOpMode {

    public static int velocity;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();
    VelocityController velController;


    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        robot.shooter.bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        velController = new VelocityController();
        robot.shooter.topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.right_trigger > 0.3){
                robot.shooter.setPower(velController.getPower(robot.shooter.getVelocity(), velocity));
            } else {
                robot.shooter.stopShooter();
            }
            if (gamepad1.x){
                robot.transfer.startIntake();
            }
            else{
                robot.transfer.stopIntake();
            }


            if (gamepad1.y){
                robot.transfer.startFeed();
            }
            else{
                robot.transfer.stopFeed();
            }
            dashboardTelemetry.addData("Current Vel (in RPM)", robot.shooter.getVelocity());
//            dashboardTelemetry.addData("Cu")
            dashboardTelemetry.update();

        }
    }
}
