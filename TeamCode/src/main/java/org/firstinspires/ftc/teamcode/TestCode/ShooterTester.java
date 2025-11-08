package org.firstinspires.ftc.teamcode.TestCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;


@Config
@TeleOp (name = "Shooter Class")
public class ShooterTester extends LinearOpMode {

    public static int velocity;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();


    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){

            if (gamepad1.right_trigger > 0.3){
                robot.shooter.setVelocity(velocity);
            } else {
                robot.shooter.stopShooter();
            }

            if (gamepad1.x){
                robot.transfer.startRamp();
            }
            else{
                robot.transfer.stopRamp();
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
