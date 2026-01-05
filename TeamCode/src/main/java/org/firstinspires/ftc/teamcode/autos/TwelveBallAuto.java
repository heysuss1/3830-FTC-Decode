package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.tasks.Tasks;


@Autonomous (name = "Red Side Auto")
public class TwelveBallAuto extends OpMode {
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
    Robot robot;
    Timer pathTimer;
    Tasks task;
    boolean hadBall, hasBall;
    double intakePathSpeed = 0.5;

    int shotCounter = 0;


    int resetCounter = 0;

    PathState pathState = PathState.TO_PRELOAD;
    ActionState actionState = ActionState.SHOOT_PRELOAD;
    Tasks.ShooterState shooterState = Tasks.ShooterState.DONE;
    boolean editingAlliance = true;





    //Starting pose wrong
    Pose startingPose, launchPose, balls1, balls2, balls3, intakeBalls1Pose, intakeBalls2Pose, intakeBalls3Pose,
    gatePose;
    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3,
            intakeBalls1, intakeBalls2, intakeBalls3, toGate;




    //This is where we set the team color;
    //If gamepad1.dpad_up, make it blue, gamepad1.dpad_down, make it red
    public void init_loop(){

    }

    public void initializePoses(Robot.Team team){
        startingPose = Robot.convertAlliancePose(new Pose(128, 118, Math.toRadians(40)), team);
        launchPose = Robot.convertAlliancePose(new Pose(96, 96, Math.toRadians(40)), team);
        balls1 = Robot.convertAlliancePose(new Pose(99, 83, 0), team);
        balls2 = Robot.convertAlliancePose(new Pose(99, 61, 0), team);
        balls3 = Robot.convertAlliancePose(new Pose(100, 35, 0), team);
        intakeBalls1Pose = Robot.convertAlliancePose(new Pose(127.5, 83, 0), team);
        intakeBalls2Pose = Robot.convertAlliancePose(new Pose(124, 61, 0), team);
        intakeBalls3Pose = Robot.convertAlliancePose(new Pose(131, 35, 0), team);
        gatePose = Robot.convertAlliancePose(new Pose(120, 70, 0), team);
    }


    //Use the values from redsideauto.java as well, but convert the x values to blue.




    public void init() {

        robot = new Robot(hardwareMap, telemetry);
        task = new Tasks(robot);
        pathTimer = new Timer();
        hasBall = !robot.intakeUptake.isUptakeEmpty();

        robot.follower.setStartingPose(startingPose);
        robot.follower.setMaxPower(1);

        while (editingAlliance){
            if (gamepad1.dpad_up) {
                Robot.setTeam(Robot.Team.BLUE);
            }
            if (gamepad1.dpad_down){
                Robot.setTeam(Robot.Team.RED);
            }
            if (gamepad1.left_bumper){
                editingAlliance = false;
            }
            telemetry.addData("Team", Robot.getTEAM());
            telemetry.update();
        }
        initializePoses(Robot.getTEAM());
        buildPaths();
    }

    public void loop(){
        hadBall = hasBall;
        hasBall = robot.intakeUptake.hasLastBall();
        autonomousUpdate();
        actionUpdate();
        task.update(hasBall, hadBall);
        robot.follower.update();
        telemetry.addData("Current Action State", actionState);
        telemetry.addData("Current Path State", pathState);
        telemetry.addData("Current Shooter State", shooterState);
        telemetry.addData("follower busy", robot.follower.isBusy());
        telemetry.addData("shooter velocity", robot.shooter.getVelocityRPM());
        telemetry.addData("shot number", shotCounter);
        telemetry.update();
    }

    public void buildPaths() {
        toPreload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchPose.getHeading())
                .build();
        toBalls1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls1.getHeading())
                .build();
        intakeBalls1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(balls1, intakeBalls1Pose))
                .setConstantHeadingInterpolation(balls1.getHeading())
                .build();

        toLaunch1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls1Pose, launchPose))
                .setLinearHeadingInterpolation(balls1.getHeading(), launchPose.getHeading())
                .build();
        toBalls2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls2.getHeading())
                .build();
        intakeBalls2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(balls2, intakeBalls2Pose))
                .setConstantHeadingInterpolation(balls2.getHeading())
                .build();
        toLaunch2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls2Pose, launchPose))
                .setLinearHeadingInterpolation(balls2.getHeading(), launchPose.getHeading())
                .build();
        toBalls3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls3.getHeading())
                .build();
        intakeBalls3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(balls3, intakeBalls3Pose))
                .setConstantHeadingInterpolation(balls3.getHeading())
                .build();
        toLaunch3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls3Pose, launchPose))
                .setLinearHeadingInterpolation(balls3.getHeading(), launchPose.getHeading())
                .build();
        toGate = robot.follower.pathBuilder()
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
                if (!robot.follower.isBusy()) {
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
                if (!robot.follower.isBusy() && pathState == PathState.SLURPING_GROUP_1){
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    setActionState(ActionState.SHOOT_GROUP_1);
                }
                break;
            case SHOOT_GROUP_1:
                if (!robot.follower.isBusy()) {
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
                if (!robot.follower.isBusy() && pathState == PathState.SLURPING_GROUP_2){
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    setActionState(ActionState.SHOOT_GROUP_2);
                }
                break;
            case SHOOT_GROUP_2:
                if (!robot.follower.isBusy()) {
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
                if (!robot.follower.isBusy() && pathState == PathState.SLURPING_GROUP_3){
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    setActionState(ActionState.SHOOT_GROUP_3);
                }
                break;
            case SHOOT_GROUP_3:
                if (!robot.follower.isBusy()) {
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
                robot.follower.followPath(toPreload, true);
                setPathState(PathState.TO_GROUP_1);
                break;
            case TO_GROUP_1:
                if (!robot.follower.isBusy() && actionState == ActionState.SLURPING_GROUP_1){
                    robot.follower.followPath(toBalls1, true);
                    setPathState(PathState.SLURPING_GROUP_1);
                }
                break;
            case SLURPING_GROUP_1:
                if (!robot.follower.isBusy() && actionState == ActionState.SHOOT_GROUP_1){
                    robot.follower.followPath(intakeBalls1,  intakePathSpeed, true);
                    setPathState(PathState.GROUP_1_TO_SHOOT);
                }
                break;
            case GROUP_1_TO_SHOOT:
                if (!robot.follower.isBusy()){
                    robot.follower.followPath(toLaunch1, true);
                    setPathState(PathState.TO_GROUP_2);
                }
                break;
            case TO_GROUP_2:
                if (!robot.follower.isBusy() && actionState == ActionState.SLURPING_GROUP_2){
                    robot.follower.followPath(toBalls2, true);
                    setPathState(PathState.SLURPING_GROUP_2);
                }
                break;
            case SLURPING_GROUP_2:
                if (!robot.follower.isBusy() && actionState == ActionState.SHOOT_GROUP_2){
                    robot.follower.followPath(intakeBalls2, 0.6, true);
                    setPathState(PathState.GROUP_2_TO_SHOOT);
                }
                break;
            case GROUP_2_TO_SHOOT:
                if (!robot.follower.isBusy()){
                    robot.follower.followPath(toLaunch2, true);
                    setPathState(PathState.TO_GROUP_3);
                }
                break;
            case TO_GROUP_3:
                if (!robot.follower.isBusy() && actionState == ActionState.SLURPING_GROUP_3){
                    robot.follower.followPath(toBalls3, true);
                    setPathState(PathState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!robot.follower.isBusy() && actionState == ActionState.SHOOT_GROUP_3){
                    robot.follower.followPath(intakeBalls3, intakePathSpeed, true);
                    setPathState(PathState.GROUP_3_TO_SHOOT);
                }
                break;
            case GROUP_3_TO_SHOOT:
                if (!robot.follower.isBusy()){
                    robot.follower.followPath(toLaunch3, true);
                    setPathState(PathState.SHOOT_TO_GATE);
                }
                break;
            case SHOOT_TO_GATE:
                if (!robot.follower.isBusy() && actionState == ActionState.STOP){
                    robot.follower.followPath(toGate);
                    setPathState(PathState.STOP);
                }
                break;
        }
    }
}
