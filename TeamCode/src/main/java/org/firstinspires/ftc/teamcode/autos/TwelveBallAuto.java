package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotConstants;


@Autonomous (name =  "12 Ball Auto")
public class TwelveBallAuto extends OpMode {

Hardware robot = Hardware.getInstance();

Follower follower;

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
                    follower.followPath(intakeBalls2, 0.6, true);
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
    //During this loop, using the DPAD, the team will be set
    public void init_loop(){RobotConstants.setTeam(RobotConstants.Team.BLUE);}


    public void init(){
    }

    public void initializeRedPaths(){

    }
    public void initializeBluePaths(){

    }

    public void loop(){
        hadBall = hasBall;
        hasBall = robot.shooter.hasBall();
        autonomousUpdate();
        actionUpdate();
        task.update(hasBall, hadBall);
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
