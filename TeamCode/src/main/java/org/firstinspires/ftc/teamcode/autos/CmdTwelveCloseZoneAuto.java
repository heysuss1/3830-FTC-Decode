package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;

public class CmdTwelveCloseZoneAuto extends AutoCommands {
    //TODO: Implement timers for shooter and pathing to timeout if something goes wrong
    private static final double intakePathSpeed = 0.6;
    private static final double AUTO_RPM = 4000;

    PathChain driveToShootPreloads, driveToGroup1, driveToShootGroup1, driveToGroup2, driveToShootGroup2,
            driveToGroup3, driveToShootGroup3, intakeGroup1, intakeGroup2, intakeGroup3, driveToGate, driveToPark;

    Pose startingPose = (new Pose(120.5, 132, Math.toRadians(45)));
    Pose shootingPose = (new Pose(96, 96, Math.toRadians(45)));
    Pose group1startPose = (new Pose(99, 85, 0));
    Pose group2startPose = (new Pose(99, 61, 0));
    Pose group3startPose = (new Pose(100, 35, 0));
    Pose group1endPose = (new Pose(127.5, 85, 0));
    Pose group2endPose = (new Pose(127, 61, 0));
    Pose group3endPose = (new Pose(131, 35, 0));
    Pose gatePose = (new Pose(120, 70, 0));
    Pose parkPose = (new Pose(120, 92, Math.PI * 3 / 2));
    private boolean isFirstTimePath = true;
    private int shotCount = 0;

    public CmdTwelveCloseZoneAuto(Robot robot, Auto.Team team, double waitTime) {
        super(robot, team, waitTime);

        startingPose = Auto.convertAlliancePose(startingPose, team);
        shootingPose = Auto.convertAlliancePose(shootingPose, team);
        group1startPose = Auto.convertAlliancePose(group1startPose, team);
        group2startPose = Auto.convertAlliancePose(group2startPose, team);
        group3startPose = Auto.convertAlliancePose(group3startPose, team);
        group1endPose = Auto.convertAlliancePose(group1endPose, team);
        group2endPose = Auto.convertAlliancePose(group2endPose, team);
        group3endPose = Auto.convertAlliancePose(group3endPose, team);
        gatePose = Auto.convertAlliancePose(gatePose, team);
        parkPose = Auto.convertAlliancePose(parkPose, team);

        robot.follower.setStartingPose(startingPose);
    }

    @Override
    public void autonomousUpdate(){
        switch (autoState) {
            case START:
                if(waitTime > 0)
                {
                    if(pathTimer.getElapsedTimeSeconds() > waitTime)
                    {
                        autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                    }
                }
                else
                {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }
                break;
            case DRIVE_TO_SHOOTING_SPOT:

                robot.shooter.setVelocityTarget(AUTO_RPM);

                if (shotCount == 0 && isFirstTimePath ) {robot.follower.followPath(driveToShootPreloads, true); isFirstTimePath = false;}
                if (shotCount == 1 && isFirstTimePath) {robot.follower.followPath(driveToShootGroup1, true); isFirstTimePath = false;}
                if (shotCount == 2 && isFirstTimePath) {robot.follower.followPath(driveToShootGroup2, true); isFirstTimePath = false;}
                if (shotCount == 3 && isFirstTimePath) {robot.follower.followPath(driveToShootGroup3, true); isFirstTimePath = false;}

                if (!robot.follower.isBusy()) { isFirstTimePath = true; autoState = AutoState.SHOOTING;}

                break;
            case SHOOTING:

                if (isFirstTimePath) {
                    robot.shooterTask.startTask(null);
                    isFirstTimePath = false;
                }

                if (robot.shooterTask.isFinished()) {
                    isFirstTimePath = true;
                    shotCount++;
                    if (shotCount <= 3) autoState = AutoState.DRIVE_TO_GROUP;
                    if (shotCount == 4) autoState = AutoState.DRIVE_TO_PARK;
                }

                break;
            case DRIVE_TO_GROUP:

                if (shotCount == 1 && isFirstTimePath) {robot.follower.followPath(driveToGroup1, true); isFirstTimePath = false;}
                if (shotCount == 2 && isFirstTimePath) {robot.follower.followPath(driveToGroup2, true); isFirstTimePath = false;}
                if (shotCount == 3 && isFirstTimePath) {robot.follower.followPath(driveToGroup3, true); isFirstTimePath = false;}

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    autoState = AutoState.SLURPING_GROUP;
                }

                break;
            case SLURPING_GROUP:

                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                if (shotCount == 1 && isFirstTimePath) {robot.follower.followPath(intakeGroup1, intakePathSpeed, true); isFirstTimePath = false;}
                if (shotCount == 2 && isFirstTimePath) {robot.follower.followPath(intakeGroup2, intakePathSpeed, true); isFirstTimePath = false;}
                if (shotCount == 3 && isFirstTimePath) {robot.follower.followPath(intakeGroup3, intakePathSpeed, true); isFirstTimePath = false;}

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }
                break;
            case DRIVE_TO_PARK:
                if (isFirstTimePath){
                    robot.follower.followPath(driveToPark);
                    isFirstTimePath = false;
                }
                if (!robot.follower.isBusy()) {
                    autoState = AutoState.STOP;
                }
            case STOP:
                cancel();
                break;
        }
    }
    @Override
    public void buildPaths() {
        driveToShootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group1startPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1startPose.getHeading())
                .build();
        intakeGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group1startPose, group1endPose))
                .setConstantHeadingInterpolation(group1startPose.getHeading())
                .build();
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group1endPose, shootingPose))
                .setLinearHeadingInterpolation(group1startPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group2startPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group2startPose.getHeading())
                .build();
        intakeGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group2startPose, group2endPose))
                .setConstantHeadingInterpolation(group2startPose.getHeading())
                .build();
        driveToShootGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group2endPose, shootingPose))
                .setLinearHeadingInterpolation(group2startPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group3startPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group3startPose.getHeading())
                .build();
        intakeGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group3startPose, group3endPose))
                .setConstantHeadingInterpolation(group3startPose.getHeading())
                .build();
        driveToShootGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group3endPose, shootingPose))
                .setLinearHeadingInterpolation(group3startPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGate = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, gatePose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), gatePose.getHeading())
                .build();
        driveToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }
}

