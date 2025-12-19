package org.firstinspires.ftc.teamcode.TestCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.RobotConstants;


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

    boolean intakeOn = false;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    public void runOpMode(){
        robot.init(hardwareMap, telemetry);
        velController = new VelocityController(hardwareMap);
        shooterTimer = new Timer();
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
                    setRobotState(RobotConstants.SystemState.INTAKING);
                    intakeOn = true;
                } else {
                    setRobotState(RobotConstants.SystemState.OFF);
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
                setRobotState(RobotConstants.SystemState.OFF);
            }


            telemetryA.addData("Current Vel (in RPM)", robot.shooter.getVelocity());
            telemetryA.addData("Color Sensor Distance", robot.shooter.colorSensor.getDistance(DistanceUnit.INCH));
            telemetryA.addData("Shot Counter",shotCounter);
            telemetryA.addData("State", systemState);
            telemetryA.update();
            updateRobotState();

        }
    }
    public void updateRobotState(){
        switch (systemState){
            case OFF:
                robot.shooter.stopShooter();
                robot.transfer.stopIntake();
                robot.transfer.stopFeed();
                shotCounter = 0;
                gamepad1.setLedColor(255,0,0,1000000000);
                break;
            case INTAKING:
                robot.transfer.setIntakeMode();
                robot.transfer.setFeedIntakeMode(robot.shooter.hasBall());
                gamepad1.setLedColor(0,0,255,1000000000);
                break;
            case SPEEDING_UP:
                //Set power to needed velocity.
                robot.shooter.setPower(velController.getPower(robot.shooter.getVelocity(), velocity));
                if (robot.shooter.isReady(velocity, 150)){
                    setRobotState(RobotConstants.SystemState.SHOOTING);
                }
                break;
            case SHOOTING:
                robot.transfer.setFeedMode();
                if (!hasBall && hadBall){
                    shotCounter ++;
                    robot.transfer.stopTransfer();
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                }
                break;
            case OUTTAKING:
                break;
            case WAITING:
                break;

        }
    }
    public void setRobotState(RobotConstants.SystemState state){
        systemState = state;
        shooterTimer.resetTimer();
    }
}
