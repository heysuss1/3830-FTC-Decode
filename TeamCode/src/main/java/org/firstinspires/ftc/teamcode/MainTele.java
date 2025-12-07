package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@Config
@TeleOp (name = "TeleOp")
public class MainTele extends LinearOpMode {

    /*
    TODO: right trigger - shoot
    TODO: left trigger - intake  (and/or left trigger)
    TODO: left trigger - aim
    TODO: square - open gate

    */
    Hardware robot = Hardware.getInstance();
    Follower follower;
    Timer shooterTimer;

    RobotConstants constants = new RobotConstants();
    Shooter.Shooter_state shooterState;
    Transfer.Transfer_state transferState;
    RobotConstants.SystemState systemState;


    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(0.8);
        double batteryVoltage = 0;
        boolean intakeOn = false;
        String team = "RED";
        int startingState = 0;
        String[] startingStateList = {"Red Back", "Red Goal", "Blue Back", "Blue Goal"};
        double[] startXposes = {60, 120, 85, 24};
        double[] startYposes = {8, 128, 8, 128};
        double[] startHeadings = {0, 36, 0, -36};
        double prevV;
        double currentV = 0;
        boolean orienting = false;
        boolean rightBumper = false;

        systemState = RobotConstants.SystemState.OFF;
        shooterState = Shooter.Shooter_state.OFF;
        transferState = Transfer.Transfer_state.OFF;

        waitForStart();
        shooterTimer = new Timer();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()){
            prevV = currentV;
            currentV = robot.turret.getDegrees();
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, follower, orienting);

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
                robot.transfer.setFeedIntakeMode();
            }

            if (currentGamepad1.dpad_up) {
                team = "RED";
            }

            if (currentGamepad1.dpad_down) {
                team = "BLUE";
            }

//            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                startingState ++;
//                if (startingState > 4) {startingState = 0;}
//                follower.setPose(new Pose(startXposes[startingState], startYposes[startingState], startHeadings[startingState]));
//            }

            orienting = currentGamepad1.b;

            if (currentGamepad1.right_trigger > 0.1)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(1);
            }

            if (currentGamepad1.x) {
                systemState = RobotConstants.SystemState.SPEEDING_UP;
            }
            if (currentGamepad1.b){
                systemState = RobotConstants.SystemState.SLOWING_DOWN;
            }

            for (VoltageSensor sensor: hardwareMap.voltageSensor){
                double voltage = sensor.getVoltage();;
                if (voltage > 0){
                    batteryVoltage = voltage;
                    break;
                }
            }
            updateRobotState();
            telemetry.addData("Starting position", startingStateList[startingState]);
            telemetry.addData("Shooter vel: ", robot.shooter.getVelocity());
            telemetry.addData("Shooter vel (raw): ", robot.shooter.shootingMotor.getVelocity());
            telemetry.addData("System state: ", systemState);
            telemetry.addData("Shooting state: ", shooterState);
            telemetry.addData("Transfer state: ", transferState);
            telemetry.addData("Shooter is ready to shoot: ", robot.shooter.isReady(3500, 40));
            telemetry.addData("Battery Voltage: ", batteryVoltage);
            telemetry.addData("Team", team);
            telemetry.addData("Rotations", robot.turret.servoRotations);
            telemetry.update();


        }
    }

    public void updateRobotState(){
        int tolerance;
        double batteryVoltage;
        switch (systemState){
            case OFF:
                robot.shooter.stopShooter();
                robot.transfer.stopRamp();
                robot.transfer.stopIntake();
                robot.transfer.stopFeed();
                break;
            case INTAKING:
                robot.transfer.setIntakeMode();
                break;
            case SPEEDING_UP:
                //Set power to needed velocity.
                robot.shooter.setVelocity(3500);
                if (robot.shooter.isReady(3350, 400)){
                    setRobotState(RobotConstants.SystemState.SHOOTING);
                }
                break;
            case SHOOTING:
                robot.transfer.setFeedMode();
                robot.transfer.startRamp();
                if (shooterTimer.getElapsedTimeSeconds() > 4){
                    setRobotState(RobotConstants.SystemState.OFF);
                }
                break;
            case OUTTAKING:
                break;
            case WAITING:
                break;

        }
    }

//    public void transferUpdate(){
//
//    }
    public void setRobotState(RobotConstants.SystemState state){
        systemState = state;
        shooterTimer.resetTimer();
    }

//    public void    public void setTransferState(Transfer.Transfer_state state) {
////        transferState = state;
////    } setTransferState(Transfer.Transfer_state state) {
//        transferState = state;
//    }
}
