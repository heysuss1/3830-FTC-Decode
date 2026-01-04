package org.firstinspires.ftc.teamcode.zArchive.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.tasks.Tasks;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Disabled
@Autonomous (name = "Red Side Auto")
public class RedSideAuto extends OpMode {
    Robot robot = Robot.getInstance();

    Follower follower;
    Timer pathTimer;
    Timer totalShooterTimer;
    Tasks task;
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
    PathState pathState = PathState.TO_PRELOAD;
    ActionState actionState = ActionState.SHOOT_PRELOAD;
    Tasks.ShooterState shooterState = Tasks.ShooterState.DONE;
    Tasks.TransferState transferState = Tasks.TransferState.OFF;


    //Starting pose wrong
    Pose startingPose = new Pose(128, 118, Math.toRadians(40));
    Pose launchPose = new Pose(96, 96, Math.toRadians(40));
    Pose balls1 = new Pose(99, 83, 0);
    Pose balls2 = new Pose(99, 61, 0);
    Pose balls3 = new Pose(100, 35, 0);
    Pose intakeBalls1Pose = new Pose(99 + FORWARD_CONSTANT+4.5, 83, 0);
    Pose intakeBalls2Pose = new Pose(99 + FORWARD_CONSTANT+1, 61, 0);
    Pose intakeBalls3Pose = new Pose(99 + FORWARD_CONSTANT+8, 35, 0);
    Pose gatePose = new Pose(120, 70, 0);
    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3,
            intakeBalls1, intakeBalls2, intakeBalls3, toGate;


    public void init() {
        robot.init(hardwareMap, telemetry);
        task = new Tasks(robot, hardwareMap, true);
        pathTimer = new Timer();
        shooterTimer = new Timer();
        totalShooterTimer = new Timer();
        hasBall = robot.shooter.hasBall();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1);
        robot.shooter.setVelocityTarget(3550.0);
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


    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();
    }
    public void setActionState(ActionState state) {
        actionState = state;
    }
    public void actionUpdate(){
        switch(actionState){
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    task.setShooterState(Tasks.ShooterState.SPEEDING_UP);
                    setActionState(ActionState.WAITING_FOR_PRELOAD);
                }
                break;
            case WAITING_FOR_PRELOAD:
                if (shooterState == Tasks.ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_1){
                    task.setTransferState(Tasks.TransferState.INTAKE);
                    setActionState(ActionState.SHOOT_GROUP_1);
                }
                break;
            case SHOOT_GROUP_1:
                if (!follower.isBusy()) {
                    task.setShooterState(Tasks.ShooterState.SPEEDING_UP);
                    setActionState(ActionState.WAITING_FOR_COMPLETION_1);
                }
                break;
            case WAITING_FOR_COMPLETION_1:
                if (shooterState == Tasks.ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_2){
                    task.setTransferState(Tasks.TransferState.INTAKE);
                    setActionState(ActionState.SHOOT_GROUP_2);
                }
                break;
            case SHOOT_GROUP_2:
                if (!follower.isBusy()) {
                    task.setShooterState(Tasks.ShooterState.SPEEDING_UP);
                    setActionState(ActionState.WAITING_FOR_COMPLETION_2);
                }
                break;
            case WAITING_FOR_COMPLETION_2:
                if (shooterState == Tasks.ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_3){
                    task.setTransferState(Tasks.TransferState.INTAKE);
                    setActionState(ActionState.SHOOT_GROUP_3);
                }
                break;
            case SHOOT_GROUP_3:
                if (!follower.isBusy()) {
                    task.setShooterState(Tasks.ShooterState.SPEEDING_UP);
                    setActionState(ActionState.WAITING_FOR_COMPLETION_3);
                }
                break;
            case WAITING_FOR_COMPLETION_3:
                if (shooterState == Tasks.ShooterState.DONE){
                    setActionState(ActionState.STOP);
                }
                break;
        }
    }


    public void autonomousUpdate(){
        switch (pathState){
            case TO_PRELOAD:
                follower.followPath(toPreload, true);
                setPathState(PathState.TO_GROUP_1);
                break;
            case TO_GROUP_1:
                if (!follower.isBusy() && actionState == ActionState.SLURPING_GROUP_1){
                    follower.followPath(toBalls1, true);
                    setPathState(PathState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!follower.isBusy() && actionState == ActionState.SHOOT_GROUP_1){
                    follower.followPath(intakeBalls1,  1, true);
                    setPathState(PathState.GROUP_1_TO_SHOOT);
                }
                break;
            case GROUP_1_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch1, true);
                    setPathState(PathState.TO_GROUP_2);
                }
                break;
            case TO_GROUP_2:
                if (!follower.isBusy() && actionState == ActionState.SLURPING_GROUP_2){
                    follower.followPath(toBalls2, true);
                    setPathState(PathState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!follower.isBusy() && actionState == ActionState.SHOOT_GROUP_2){
                    follower.followPath(intakeBalls2, 1, true);
                    setPathState(PathState.GROUP_2_TO_SHOOT);
                }
                break;
            case GROUP_2_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch2, true);
                    setPathState(PathState.TO_GROUP_3);
                }
                break;
            case TO_GROUP_3:
                if (!follower.isBusy() && actionState == ActionState.SLURPING_GROUP_3){
                    follower.followPath(toBalls3, true);
                    setPathState(PathState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && actionState == ActionState.SHOOT_GROUP_3){
                    follower.followPath(intakeBalls3, 1, true);
                    setPathState(PathState.GROUP_3_TO_SHOOT);
                }
                break;
            case GROUP_3_TO_SHOOT:
                if (!follower.isBusy()){
                    follower.followPath(toLaunch3, true);
                    setPathState(PathState.SHOOT_TO_GATE);
                }
                break;
            case SHOOT_TO_GATE:
                if (!follower.isBusy() && actionState == ActionState.STOP){
                    follower.followPath(toGate);
                    setPathState(PathState.STOP);
                }
                break;
        }
    }

    public void loop(){
        hadBall = hasBall;
        hasBall = robot.shooter.hasBall();
        shooterState = task.getShooterState();
        transferState = task.getTransferState();
        autonomousUpdate();
        actionUpdate();
        task.update(hasBall, hadBall);
        follower.update();
        telemetry.addData("Current Action State", actionState);
        telemetry.addData("Current Path State", pathState);
        telemetry.addData("Current Shooter State", shooterState);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("shooter timer", shooterTimer.getElapsedTimeSeconds());
        telemetry.addData("shooter velocity", robot.shooter.getVelocity());
        telemetry.addData("battery voltage", velController.getBatteryVoltage());
        telemetry.addData("shot number", shotCounter);
        telemetry.update();
    }
}
