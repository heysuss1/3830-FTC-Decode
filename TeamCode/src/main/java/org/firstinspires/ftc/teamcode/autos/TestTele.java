package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous (name = "Funny Test TeleOp")
public class TestTele extends OpMode {
    Follower follower;
    Timer pathTimer;

    Pose startingPose = new Pose(88,135, 0);
    Pose launchPose = new Pose(93, 110, Math.toRadians(25));
    Pose balls1 = new Pose(99, 83, 0);
    Pose balls2 = new Pose(99, 60, 0);
    Pose balls3 = new Pose(100, 35, 0);

    int pathState = 0;
    PathChain toPreload, toBalls1, toLaunch1, toBalls2, toLaunch2, toBalls3, toLaunch3;
    public void init(){
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildPaths();
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

    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();

    }

    public void autonomousUpdate(){
        switch (pathState){
            case 0:
                follower.followPath(toPreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5){
                    follower.followPath(toBalls1, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5){
                    follower.followPath(toLaunch1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5){
                    follower.followPath(toBalls2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3.5){
                    follower.followPath(toLaunch2, true);
                    setPathState(5);
                }
                break;
        }
    }
    public void loop(){
        autonomousUpdate();
        follower.update();
    }
}
