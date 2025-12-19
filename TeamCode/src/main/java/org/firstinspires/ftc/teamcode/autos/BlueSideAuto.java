package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous (name = "Blue Side Auto")
public class BlueSideAuto extends OpMode {
    Hardware robot = Hardware.getInstance();

    Follower follower;
    Timer pathTimer;
    Timer totalShooterTimer;
    boolean hadBall, hasBall;
    double intakePathSpeed = 0.5;
    double batteryVoltage;
    Timer shooterTimer;
    VelocityController velController;
    int shotCounter = 0;

    final int FORWARD_CONSTANT = 24;

    enum ActionState {
        SHOOT_PRELOAD,
        WAITING_FOR_PRELOAD,
        SLURPING_GROUP_1,
        SHOOT_GROUP_1,
        WAITING_FOR_COMPLETION_1,
        SLURPING_GROUP_2,
        SHOOT_GROUP_2,
        WAITING_FOR_COMPLETION_2,
        SLURPING_GROUP_3,
        SHOOT_GROUP_3,
        WAITING_FOR_COMPLETION_3,
        STOP,
    }

    int resetCounter = 0;
    enum PathState {
        TO_PRELOAD,
        TO_GROUP_1,
        SLURPING_GROUP_1,
        GROUP_1_TO_SHOOT,
        TO_GROUP_2,
        SLURPING_GROUP_2,
        GROUP_2_TO_SHOOT,
        TO_GROUP_3,
        SLURPING_GROUP_3,
        GROUP_3_TO_SHOOT,
        SHOOT_TO_GATE,
        STOP,
    }

    enum ShooterState {
        SPEED_UP,
        FEED_BALLS,
        DONE
    }

    RedSideAuto.PathState pathState = RedSideAuto.PathState.TO_PRELOAD;
    RedSideAuto.ActionState actionState = RedSideAuto.ActionState.SHOOT_PRELOAD;
    RobotConstants.SystemState robotState = RobotConstants.SystemState.OFF;
    Pose startingPose = new Pose(convertRedToBluePosition(128), 118, Math.toRadians(140));
    Pose launchPose = new Pose((convertRedToBluePosition(96)), 96, Math.toRadians(149.4));

    Pose balls1 = new Pose(convertRedToBluePosition(99), 78, Math.toRadians(180));
    Pose balls2 = new Pose(convertRedToBluePosition(99)-4, 57, Math.toRadians(182));
    Pose balls3 = new Pose(convertRedToBluePosition(100)-5, 33, Math.toRadians(182));
    Pose intakeBalls1Pose = new Pose(convertRedToBluePosition(99 + FORWARD_CONSTANT+3.5), 78, Math.toRadians(182));
    Pose intakeBalls2Pose = new Pose(convertRedToBluePosition(99 + FORWARD_CONSTANT+1), 57, Math.toRadians(182));
    Pose intakeBalls3Pose = new Pose(convertRedToBluePosition(99 + FORWARD_CONSTANT+8), 33, Math.toRadians(180));
    Pose gatePose = new Pose(convertRedToBluePosition(120), 70, Math.toRadians(180));
    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3,
            intakeBalls1, intakeBalls2, intakeBalls3, toGate;



    public double convertRedToBluePosition(double position){
        return position - 2 * (position - 75);
    }

    public void init() {
        robot.init(hardwareMap, telemetry);
        pathTimer = new Timer();
        shooterTimer = new Timer();
        totalShooterTimer = new Timer();
        hasBall = robot.shooter.hasBall();
        velController = new VelocityController(hardwareMap);
        batteryVoltage = velController.getBatteryVoltage();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1);
        buildPaths();
    }

    public void buildPaths() {
        toPreload = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchPose.getHeading())
                .build();
        toBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls1.getHeading())
                .build();
        intakeBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(balls1, intakeBalls1Pose))
                .setConstantHeadingInterpolation(balls1.getHeading())
                .build();

        toLaunch1 = follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls1Pose, launchPose))
                .setLinearHeadingInterpolation(balls1.getHeading(), launchPose.getHeading())
                .build();
        toBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls2.getHeading())
                .build();
        intakeBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(balls2, intakeBalls2Pose))
                .setConstantHeadingInterpolation(balls2.getHeading())
                .build();
        toLaunch2 = follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls2Pose, launchPose))
                .setLinearHeadingInterpolation(balls2.getHeading(), launchPose.getHeading())
                .build();
        toBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls3.getHeading())
                .build();
        intakeBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(balls3, intakeBalls3Pose))
                .setConstantHeadingInterpolation(balls3.getHeading())
                .build();
        toLaunch3 = follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls3Pose, launchPose))
                .setLinearHeadingInterpolation(balls3.getHeading(), launchPose.getHeading())
                .build();
        toGate = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, gatePose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), gatePose.getHeading())
                .build();
    }
    public void shooterUpdate() {
        switch (robotState){
            case OFF:
                robot.shooter.stopShooter();
                robot.transfer.stopIntake();
                robot.transfer.stopFeed();
                totalShooterTimer.resetTimer();
                shotCounter = 0;
                batteryVoltage = velController.getBatteryVoltage();
                break;
            case INTAKING:
                robot.transfer.setIntakeMode();
                robot.transfer.setFeedIntakeMode(robot.shooter.hasBall());
                batteryVoltage = velController.getBatteryVoltage();
                break;
            case SPEEDING_UP:
                //Set power to needed velocity.
                robot.shooter.setPower(velController.getPower(robot.shooter.getVelocity(), 3600));
                if (robot.shooter.isReady(3600, 150) || shooterTimer.getElapsedTimeSeconds() > 1.25){ //TODO: Why 300 why not 100
                    setRobotState(RobotConstants.SystemState.SHOOTING);
                }
                break;
             case SHOOTING:
                if (resetCounter == 0){
                    totalShooterTimer.resetTimer();
                    resetCounter++;
                }
                if (shotCounter >= 3 || totalShooterTimer.getElapsedTimeSeconds() > 6){
                    setRobotState(RobotConstants.SystemState.OFF);
                }
                robot.transfer.setFeedMode();
                if (hadBall && !hasBall){
                    robot.transfer.stopTransfer();
                    shotCounter++;
                    setRobotState(RobotConstants.SystemState.WAITING_FOR_SHOT);
                }
                break;
            case WAITING_FOR_SHOT:
                if (shooterTimer.getElapsedTimeSeconds() > 0.6){
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
        robotState = state;
        shooterTimer.resetTimer();
    }


    public void setPathState(RedSideAuto.PathState state){
        pathState = state;
        pathTimer.resetTimer();
    }
    public void setActionState(RedSideAuto.ActionState state) {
        actionState = state;
    }
    public void actionUpdate(){
        switch(actionState){
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                    setActionState(RedSideAuto.ActionState.WAITING_FOR_PRELOAD);
                }
                break;
            case WAITING_FOR_PRELOAD:
                if (robotState == RobotConstants.SystemState.OFF){
                    setActionState(RedSideAuto.ActionState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!follower.isBusy() && pathState == RedSideAuto.PathState.SLURPING_GROUP_1){
                    setRobotState(RobotConstants.SystemState.INTAKING);
                    setActionState(RedSideAuto.ActionState.SHOOT_GROUP_1);
                }
                break;
            case SHOOT_GROUP_1:
                if (!follower.isBusy()) {
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                    setActionState(RedSideAuto.ActionState.WAITING_FOR_COMPLETION_1);
                }
                break;
            case WAITING_FOR_COMPLETION_1:
                if (robotState == RobotConstants.SystemState.OFF){
                    setActionState(RedSideAuto.ActionState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!follower.isBusy() && pathState == RedSideAuto.PathState.SLURPING_GROUP_2){
                    setRobotState(RobotConstants.SystemState.INTAKING);
                    setActionState(RedSideAuto.ActionState.SHOOT_GROUP_2);
                }
                break;
            case SHOOT_GROUP_2:
                if (!follower.isBusy()) {
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                    setActionState(RedSideAuto.ActionState.WAITING_FOR_COMPLETION_2);
                }
                break;
            case WAITING_FOR_COMPLETION_2:
                if (robotState == RobotConstants.SystemState.OFF){
                    setActionState(RedSideAuto.ActionState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && pathState == RedSideAuto.PathState.SLURPING_GROUP_3){
                    setRobotState(RobotConstants.SystemState.INTAKING);
                    setActionState(RedSideAuto.ActionState.SHOOT_GROUP_3);
                }
                break;
            case SHOOT_GROUP_3:
                if (!follower.isBusy()) {
                    setRobotState(RobotConstants.SystemState.SPEEDING_UP);
                    setActionState(RedSideAuto.ActionState.WAITING_FOR_COMPLETION_3);
                }
                break;
            case WAITING_FOR_COMPLETION_3:
                if (robotState == RobotConstants.SystemState.OFF){
                    setActionState(RedSideAuto.ActionState.STOP);
                }
                break;
        }
    }


    public void autonomousUpdate(){
        switch (pathState){
            case TO_PRELOAD:
                follower.followPath(toPreload, true);
                setPathState(RedSideAuto.PathState.TO_GROUP_1);
                break;
            case TO_GROUP_1:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SLURPING_GROUP_1){
                    follower.followPath(toBalls1, true);
                    setPathState(RedSideAuto.PathState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SHOOT_GROUP_1){
                    follower.followPath(intakeBalls1,  intakePathSpeed, true);
                    setPathState(RedSideAuto.PathState.GROUP_1_TO_SHOOT);
                }
                break;
            case GROUP_1_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch1, true);
                    setPathState(RedSideAuto.PathState.TO_GROUP_2);
                }
                break;
            case TO_GROUP_2:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SLURPING_GROUP_2){
                    follower.followPath(toBalls2, true);
                    setPathState(RedSideAuto.PathState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SHOOT_GROUP_2){
                    follower.followPath(intakeBalls2, 0.3, true);
                    setPathState(RedSideAuto.PathState.GROUP_2_TO_SHOOT);
                }
                break;
            case GROUP_2_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch2, true);
                    setPathState(RedSideAuto.PathState.TO_GROUP_3);
                }
                break;
            case TO_GROUP_3:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SLURPING_GROUP_3){
                    follower.followPath(toBalls3, true);
                    setPathState(RedSideAuto.PathState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.SHOOT_GROUP_3){
                    follower.followPath(intakeBalls3, intakePathSpeed, true);
                    setPathState(RedSideAuto.PathState.GROUP_3_TO_SHOOT);
                }
                break;
            case GROUP_3_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch3, true);
                    setPathState(RedSideAuto.PathState.SHOOT_TO_GATE);
                }
                break;
            case SHOOT_TO_GATE:
                if (!follower.isBusy() && actionState == RedSideAuto.ActionState.STOP){
                    follower.followPath(toGate);
                    setPathState(RedSideAuto.PathState.STOP);
                }
                break;
        }
    }

    public void loop(){
        hadBall = hasBall;
        hasBall = robot.shooter.hasBall();
        autonomousUpdate();
        actionUpdate();
        shooterUpdate();
        follower.update();
        telemetry.addData("Current Action State", actionState);
        telemetry.addData("Current Path State", pathState);
        telemetry.addData("Current Shooter State", robotState);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("shooter timer", shooterTimer.getElapsedTimeSeconds());
        telemetry.addData("shooter velocity", robot.shooter.getVelocity());
        telemetry.addData("battery voltage", velController.getBatteryVoltage());
        telemetry.addData("shot number", shotCounter);
        telemetry.update();
    }
}

