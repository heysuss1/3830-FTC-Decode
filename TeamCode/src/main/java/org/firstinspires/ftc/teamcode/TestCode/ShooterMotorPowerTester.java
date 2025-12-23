package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp (name = "Shooter Velocity Checker")
public class ShooterMotorPowerTester extends LinearOpMode {

    public static double power;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetryA;


    Hardware robot = Hardware.getInstance();
    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            robot.shooter.setPower(power);
            telemetryA.addData("top shooter velocity (rpm)", robot.shooter.getVelocity());
            telemetryA.update();
        }
    }
}
