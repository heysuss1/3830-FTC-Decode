package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.tasks.Tasks;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp (name = "Test Turret Rotation")
@Config
public class DegreesToTicksTesterMotor extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static int targetDegree;
    Follower follower;
    public static double position;
    Tasks task;
    public double gearRatio = 0.3819;
    Hardware robot = Hardware.getInstance();

    public final double TICKS_PER_REV = 142.8;
    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        task = new Tasks(robot, hardwareMap, false);
        Constants.createFollower(hardwareMap);
        waitForStart();
        double degrees;
        int ticks;
        double robotHeading;
        while(opModeIsActive()){
            robot.shooter.pitchServo.setPosition(position);

            ticks = robot.shooter.getTurretPosition();
            degrees = (ticks/TICKS_PER_REV * gearRatio) * 360;



            follower.update();
            dashboardTelemetry.addData("turret ticks", ticks);
            dashboardTelemetry.addData("Degrees", degrees);
            dashboardTelemetry.addData("Degrees in Ticks", robot.shooter.degreesToTicks(targetDegree));
            dashboardTelemetry.addData("Robot's heading", Math.toDegrees(follower.getHeading()));
            dashboardTelemetry.update();
        }
    }
}
