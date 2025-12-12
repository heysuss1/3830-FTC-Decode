package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VelocityController;


@Autonomous (name = "Red Side Auto")
public class RedSideAuto extends OpMode {
    Hardware robot = Hardware.getInstance();

    Follower follower;
    Timer pathTimer;
    double intakePathSpeed = 0.5;
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

    PathState pathState = PathState.TO_PRELOAD;
    ActionState actionState = ActionState.SHOOT_PRELOAD;
    ShooterState shooterState = ShooterState.SPEED_UP;

    Pose startingPose = new Pose(88, 135, 0);
    Pose launchPose = new Pose(93, 110, Math.toRadians(25));
    Pose balls1 = new Pose(99, 83, 0);
    Pose balls2 = new Pose(99, 60, 0);
    Pose balls3 = new Pose(100, 35, 0);
    Pose intakeBalls1Pose = new Pose(99 + FORWARD_CONSTANT+3, 83, 0);
    Pose intakeBalls2Pose = new Pose(99 + FORWARD_CONSTANT, 60, 0);
    Pose intakeBalls3Pose = new Pose(99 + FORWARD_CONSTANT+6, 35, 0);
    Pose gatePose = new Pose(125, 70, 0);
    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3,
            intakeBalls1, intakeBalls2, intakeBalls3, toGate;


    public void init() {
        robot.init(hardwareMap, telemetry);
        pathTimer = new Timer();
        shooterTimer = new Timer();
        velController = new VelocityController();
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
        switch (shooterState) {
            case SPEED_UP:
                //Set power to needed velocity.
                robot.shooter.setPower(velController.getPower(robot.shooter.getVelocity(), 4250));
                if (robot.shooter.isReady(3600 , 450)){
                    setShooterState(ShooterState.FEED_BALLS);
                }
                break;
            case FEED_BALLS:
                robot.transfer.setFeedMode();
                if (shooterTimer.getElapsedTimeSeconds() > 0.4){
                    shotCounter++;
                    robot.transfer.stopTransfer();
                    setShooterState(ShooterState.SPEED_UP);
                }
                if (shotCounter >= 3){
                    setShooterState(ShooterState.DONE);
                }
                break;
            case DONE:
                shotCounter = 0;
                break;
        }
    }

    public void setShooterState(ShooterState state){
        shooterState = state;
        shooterTimer.resetTimer();
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
                    setShooterState(ShooterState.SPEED_UP);
                    setActionState(ActionState.WAITING_FOR_PRELOAD);
                }
                break;
            case WAITING_FOR_PRELOAD:
                if (shooterState == ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_1){
                    robot.transfer.setIntakeMode();
                    setActionState(ActionState.SHOOT_GROUP_1);
                }
                break;
            case SHOOT_GROUP_1:
                if (!follower.isBusy()) {
                    setShooterState(ShooterState.SPEED_UP);
                    setActionState(ActionState.WAITING_FOR_COMPLETION_1);
                }
                break;
            case WAITING_FOR_COMPLETION_1:
                if (shooterState == ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_2){
                    robot.transfer.setIntakeMode();
                    setActionState(ActionState.SHOOT_GROUP_2);
                }
                break;
            case SHOOT_GROUP_2:
                if (!follower.isBusy()) {
                    setShooterState(ShooterState.SPEED_UP);
                    setActionState(ActionState.WAITING_FOR_COMPLETION_2);
                }
                break;
            case WAITING_FOR_COMPLETION_2:
                if (shooterState == ShooterState.DONE){
                    setActionState(ActionState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && pathState == PathState.SLURPING_GROUP_3){
                    robot.transfer.setIntakeMode();
                    setActionState(ActionState.SHOOT_GROUP_3);
                }
                break;
            case SHOOT_GROUP_3:
                if (!follower.isBusy()) {
                    setActionState(ActionState.WAITING_FOR_COMPLETION_3);
                    setShooterState(ShooterState.SPEED_UP);
                }
                break;
            case WAITING_FOR_COMPLETION_3:
                if (shooterState == ShooterState.DONE){
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
                    follower.followPath(intakeBalls1,  intakePathSpeed, true);
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
                    follower.followPath(intakeBalls2, 0.3, true);
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
                    follower.followPath(intakeBalls3, intakePathSpeed, true);
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
        autonomousUpdate();
        actionUpdate();
        shooterUpdate();
        follower.update();
        telemetry.addData("Current Action State", actionState);
        telemetry.addData("Current Path State", pathState);
        telemetry.addData("Current Shooter State", shooterState);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("shooter timer", shooterTimer.getElapsedTimeSeconds());
        telemetry.addData("shooter velocity", robot.shooter.getVelocity());

        telemetry.update();
    }
}
