package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;

public class CmdFarZoneAuto extends AutoCommands{

    PathChain driveToShootGroup1;
    Pose startPose = new Pose(80, 8, Math.PI/2);
    Pose humanPlayerPickUpPose = new Pose(135, 8, 0);

    public CmdFarZoneAuto(Robot robot, Auto.Team team, Auto.AutoStrategy autoStrategy,double waitTime){
        super(robot, team, autoStrategy, waitTime);
        startPose = Auto.convertAlliancePose(startPose, team);
        humanPlayerPickUpPose = Auto.convertAlliancePose(humanPlayerPickUpPose, team);
        robot.follower.setPose(startPose);

    }

    @Override
    public void autonomousUpdate(){
        switch (autoState) {
            case START:
                if (waitTime <= 0 || timer.getElapsedTimeSeconds() > waitTime) {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }
                break;

            case DRIVE_TO_SHOOTING_SPOT:

                robot.shooter.setVelocityTarget(AUTO_RPM);

                if (shotCount == 0 && isFirstTimePath ) setAutoState(AutoState.SHOOTING);
                if (shotCount == 1 && isFirstTimePath) { robot.follower.followPath(driveToShootGroup1, true); isFirstTimePath = false;}

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    setAutoState(AutoState.SHOOTING);
                }
                break;
            case SHOOTING:

                if (isFirstTimePath){
                    robot.shooterTask.startTask(null);
                    isFirstTimePath = false;
                }

                if (robot.shooterTask.isFinished()) {
                    isFirstTimePath = true;
                    shotCount++;
                    setAutoState(AutoState.DRIVE_TO_GROUP);
                }

                break;
            case DRIVE_TO_GROUP:
                if (shotCount == 1 && isFirstTimePath) robot.follower.followPath(driveToShootGroup1); isFirstTimePath = false;

                if (!robot.follower.isBusy()){
                    isFirstTimePath = true;
                    setAutoState(AutoState.STOP);
                }
                break;
            case STOP:
                cancel();
                break;
        }
    }

    @Override
    public void buildPaths(){
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPickUpPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), humanPlayerPickUpPose.getHeading())
                .build();
    }
}
