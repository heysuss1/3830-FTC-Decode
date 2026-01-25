package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public class CmdFarZoneAuto extends AutoCommands{

    boolean isFirstTimePath = true;
    Pose startPose = new Pose(80, 8, Math.PI/2);
    Pose humanPlayerPickUpPose = new Pose(135, 8, 0);

    PathChain driveToShootGroup1;
    final double AUTO_RPM = 4000;
    int shotCount = 0;
    @Override
    public void buildPaths(){
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPickUpPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), humanPlayerPickUpPose.getHeading())
                .build();
    }

    public CmdFarZoneAuto(Robot robot, Auto.Team team, double waitTime){
        super(robot, team, waitTime);
        startPose = Auto.convertAlliancePose(startPose, team);
        humanPlayerPickUpPose = Auto.convertAlliancePose(humanPlayerPickUpPose, team);
        robot.follower.setPose(startPose);

    }
    @Override
    public void autonomousUpdate(){
        switch (autoState) {
            case START:
                //intentionally fall through!!111!11111111!!1
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
                    robot.shooterTask.startTask();
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
        }
    }
}
