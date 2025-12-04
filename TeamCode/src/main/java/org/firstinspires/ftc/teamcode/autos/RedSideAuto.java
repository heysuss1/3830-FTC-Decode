package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous (name = "Red Side Auto")
public class RedSideAuto extends OpMode {
    Follower follower;
    Timer pathTimer;


    enum ActionState{
        SHOOT_PRELOAD,
        SLURPING_GROUP_1,
        SHOOT_GROUP_1,
        SLURPING_GROUP_2,
        SHOOT_GROUP_2,
        SLURPING_GROUP_3,
        SHOOT_GROUP_3,
        STOP,


    }
    enum PathState{
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

    enum ShooterState{
        SPEED_UP,
        FEED_BALLS,
        DONE
    }

    PathState pathState = PathState.TO_PRELOAD;
    ActionState actionState = ActionState.SHOOT_PRELOAD;
    ShooterState shooterState = ShooterState.SPEED_UP;


    Pose startingPose = new Pose(88,135, 0);
    Pose launchPose = new Pose(93, 110, Math.toRadians(25));
    Pose balls1 = new Pose(99, 83, 0);
    Pose balls2 = new Pose(99, 60, 0);
    Pose balls3 = new Pose(100, 35, 0);

    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3;
    public void init(){
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildPaths();
    }

    public void shooterUpdate(){
        switch(shooterState){

        }
    }

    public void setShooterState(ShooterState state){
        shooterState = state;

    }
    public void buildPaths(){
        toPreload = follower.pathBuilder()
                .addPath(new BezierLine(startingPose, launchPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), launchPose.getHeading())
                .build();
        toBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls1.getHeading())
                .build();
        toLaunch1 = follower.pathBuilder()
                .addPath(new BezierLine(balls1, launchPose))
                .setLinearHeadingInterpolation(balls1.getHeading(), launchPose.getHeading())
                .build();
        toBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls2.getHeading())
                .build();
        toLaunch2 = follower.pathBuilder()
                .addPath(new BezierLine(balls2, launchPose))
                .setLinearHeadingInterpolation(balls2.getHeading(), launchPose.getHeading())
                .build();
        toBalls3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, balls3))
                .setLinearHeadingInterpolation(launchPose.getHeading(), balls3.getHeading())
                .build();
        toLaunch3 = follower.pathBuilder()
                .addPath(new BezierLine(balls3, launchPose))
                .setLinearHeadingInterpolation(balls3.getHeading(), launchPose.getHeading())
                .build();

    }

    public void setPathState(PathState state){
        pathState = state;
        pathTimer.resetTimer();
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
                    //TODO: ceate path to go through balls
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
//                    follower.followPath(toLaunch2, true);
                    //TODO: create path!
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
//                    follower.followPath(toLaunch2, true);
                    //TODO make later
                    setPathState(PathState.SLURPING_GROUP_3);
                }
                break;
            case SLURPING_GROUP_3:
                if (!follower.isBusy() && actionState == ActionState.SHOOT_GROUP_3){
//                    follower.followPath(toLaunch3, true);
                    //TODO make later
                    setPathState(PathState.GROUP_3_TO_SHOOT);
                }
                break;
            case GROUP_3_TO_SHOOT:
                if (!follower.isBusy()){
//                    follower.followPath(toLaunch3, true);
                    //TODO make later
                    setPathState(PathState.SHOOT_TO_GATE);
                }
                break;
            case SHOOT_TO_GATE:
                if (!follower.isBusy() && actionState == ActionState.STOP){
//                    follower.followPath(toLaunch3, true);
                    //TODO make later
                    setPathState(PathState.STOP);
                }
                break;


        }
    }
    public void loop(){
        autonomousUpdate();
        follower.update();
    }
}
