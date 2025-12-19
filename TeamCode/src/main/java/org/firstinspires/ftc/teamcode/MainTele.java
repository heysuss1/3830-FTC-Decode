package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.subsystems.Alignment;

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

    VelocityController velController;
    RobotConstants constants = new RobotConstants();
    Shooter.Shooter_state shooterState;
    Transfer.Transfer_state transferState;
    RobotConstants.SystemState systemState;
    double batteryVoltage;
    int shotCounter = 0;
    int targetVel = 3600;
    boolean hadBall, hasBall;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(142,54,Math.PI));
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(0.8);
        velController  =  new VelocityController(hardwareMap);
        batteryVoltage = velController.getBatteryVoltage();
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

        while (opModeIsActive()){
            hadBall = hasBall;
            hasBall = robot.shooter.hasBall();

            prevV = currentV;
            currentV = robot.shooter.getVelocity();
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
                robot.transfer.startFeed();
            }

            if (currentGamepad1.circle){
                robot.transfer.moveBackwards();
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


            if (currentGamepad1.right_trigger > 0.1 || currentGamepad1.x)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(1);
            }

//            if (currentGamepad1.x) {
//                if (Alignment.yawAligned(follower.getPose().getX(), follower.getPose().getY(), 142,140.4, follower.getHeading())) {
//                        systemState = RobotConstants.SystemState.SPEEDING_UP;
//                } else {
//                    Alignment.alignYaw(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), 142, 140.4);
//                }
//            }

            if (currentGamepad1.x && !previousGamepad1.x){
                shooterTimer.resetTimer();
                systemState = RobotConstants.SystemState.SPEEDING_UP;
            }

            if (currentGamepad1.b){
                systemState = RobotConstants.SystemState.SLOWING_DOWN;
            }
            if (currentGamepad1.cross){
                setRobotState(RobotConstants.SystemState.OFF);
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                targetVel += 100;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                targetVel -= 100;
            }

//            for (VoltageSensor sensor: hardwareMap.voltageSensor){
//                double voltage = sensor.getVoltage();;
//                if (voltage > 0){
//                    batteryVoltage = voltage;
//                    break;
//                }
//            }
            updateRobotState();
            telemetry.addData("Shooter vel: ", robot.shooter.getVelocity());
            telemetry.addData("Shooter vel (raw): ", robot.shooter.topShooterMotor.getVelocity());
            telemetry.addData("Target Vel", targetVel);
//            telemetry.addData("Shooter is ready to shoot: ", robot.shooter.isReady(3500, 40));
            telemetry.addData("Team", team);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("Heading: ", follower.getHeading());
            telemetry.addData("Shooter Timer", shooterTimer.getElapsedTimeSeconds());
            telemetry.addData("Shooter state", systemState);
            telemetry.addData("yawAligned?: ", Alignment.yawAligned(follower.getPose().getX(), follower.getPose().getY(), 12,140.4, follower.getHeading()));
            telemetry.update();
            follower.update();


        }
    }

    public void updateRobotState(){
        int tolerance;
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
                if (robot.shooter.hasBall()){
                    currentGamepad1.rumble(1000);
                }
                robot.transfer.setFeedIntakeMode(robot.shooter.hasBall());
                gamepad1.setLedColor(0,0,255,1000000000);
                batteryVoltage = velController.getBatteryVoltage();
                break;
            case SPEEDING_UP:
                robot.shooter.setPower(velController.getPower(robot.shooter.getVelocity(), targetVel));
                if (robot.shooter.isReady(targetVel, 150) || shooterTimer.getElapsedTimeSeconds() > 1.5 ){ //TODO: Why 300 why not 100
                    setRobotState(RobotConstants.SystemState.SHOOTING);
                    return;
                }
                break;
            case SHOOTING:
                robot.transfer.setFeedMode();
                if (shooterTimer.getElapsedTimeSeconds() > 0.28){
                    robot.transfer.stopTransfer();
                    shotCounter ++;
                    setRobotState(RobotConstants.SystemState.WAITING_FOR_SHOT);
                    return;
                }

                if (shotCounter >= 3){
                    robot.transfer.stopTransfer();
                    setRobotState(RobotConstants.SystemState.OFF);
                    return;
                }
//                if (hadBall && !hasBall){
//                    robot.transfer.stopTransfer();
//                    shotCounter++;
//                    setRobotState(RobotConstants.SystemState.WAITING_FOR_SHOT);
//                    return;
//                }
                break;
            case WAITING_FOR_SHOT:
                if (shooterTimer.getElapsedTimeSeconds() >.8){
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                    return;
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
