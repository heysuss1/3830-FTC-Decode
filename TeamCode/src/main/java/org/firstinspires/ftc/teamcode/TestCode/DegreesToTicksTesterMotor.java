package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp (name = "AmeliaSadness")
@Config
public class DegreesToTicksTesterMotor extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static int targetDegree;
    public static double position;
    public double gearRatio = 0.3819;
    ArcSortingBot robot = ArcSortingBot.getInstance();

    public final double TICKS_PER_REV = 142.8;
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        double degrees;
        int ticks;
        double robotHeading;
        while(opModeIsActive()){
            robotHeading = robot.pinpoint.getHeading(AngleUnit.DEGREES);
            robot.shooter.pitchServo.setPosition(position);
            robot.turret.turretMotor.setTargetPosition(robot.turret.degreesToTicks(targetDegree + (-robotHeading)));
            robot.turret.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.turretMotor.setPower(0.1);
            ticks = robot.turret.turretMotor.getCurrentPosition();
            degrees = (ticks/TICKS_PER_REV * gearRatio) * 360;



            
            dashboardTelemetry.addData("turret ticks",robot.turret.turretMotor.getCurrentPosition());
            dashboardTelemetry.addData("Degrees", degrees);
            dashboardTelemetry.addData("Degrees in Ticks", robot.turret.degreesToTicks(targetDegree));
            dashboardTelemetry.addData("Robot's heading", robotHeading);
            dashboardTelemetry.update();
            robot.pinpoint.update();
        }
    }
}
