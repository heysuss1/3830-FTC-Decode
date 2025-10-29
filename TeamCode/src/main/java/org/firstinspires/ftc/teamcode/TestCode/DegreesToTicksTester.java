package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp (name = "AmeliaSadness")
@Config
public class DegreesToTicksTester extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();

    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            robot.turret.turretRotation.getCurrentPosition();
            telemetry.addData("turret ticks",robot.turret.turretRotation.getCurrentPosition());
            telemetry.update();
        }
    }
}
