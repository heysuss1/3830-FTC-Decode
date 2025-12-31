package org.firstinspires.ftc.teamcode.testCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.controllers.VelocityController;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.tasks.Tasks;


@Config
@TeleOp (name = "Shooter Tester")
public class ShooterTester extends LinearOpMode {

    public static int velocity;

    com.pedropathing.util.Timer shooterTimer;
    int shotCounter = 0;
    Telemetry telemetryA;
    RobotConstants.SystemState systemState = RobotConstants.SystemState.OFF;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean hadBall, hasBall;
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();
    VelocityController velController;
    Tasks tasks;

    boolean intakeOn = false;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        velController = new VelocityController(hardwareMap);
        shooterTimer = new Timer();
        tasks = new Tasks(robot, hardwareMap, false);
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        waitForStart();
        hasBall = robot.shooter.hasBall();
        while(opModeIsActive()){
            hadBall = hasBall;
            hasBall = robot.shooter.hasBall();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                if (!intakeOn){
                    tasks.setTransferState(Tasks.TransferState.INTAKE);
                    intakeOn = true;
                } else {
                    tasks.setTransferState(Tasks.TransferState.OFF);
                    intakeOn = false;
                }
            }
            if (currentGamepad1.y) {
                robot.transfer.startFeed();
            }
            if (currentGamepad1.x){
                systemState = RobotConstants.SystemState.SPEEDING_UP;
            }

            if (currentGamepad1.b){
                systemState = RobotConstants.SystemState.SLOWING_DOWN;
            }
            if (currentGamepad1.cross){
                tasks.setTransferState(Tasks.TransferState.OFF);
                tasks.setShooterState(Tasks.ShooterState.DONE);
            }


            tasks.update(hasBall, hadBall);
            telemetryA.addData("Current Vel (in RPM)", robot.shooter.getVelocity());
            telemetryA.addData("Color Sensor Distance", robot.shooter.colorSensor.getDistance(DistanceUnit.INCH));
            telemetryA.addData("Shot Counter",shotCounter);
            telemetryA.addData("State", systemState);
            telemetryA.update();
        }
    }
}
