package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;

public class CmdCloseZoneAuto extends AutoCommands {

    PathChain driveToShootPreloads, driveToGroup1, driveToShootGroup1, driveToGroup2, driveToShootGroup2,
            driveToGroup3, driveToShootGroup3, intakeGroup1, intakeGroup2, intakeGroup3, driveToPark,
            driveToOpenGate, intakeFromGate, driveToShootGateGroup;

    Pose startingPose = (new Pose(118.8, 127.9, Math.toRadians(36)));
    Pose shootingPose = (new Pose(93, 78, 0));
    Pose group2startPose = (new Pose(105, 64, Math.toRadians(-10)));
    Pose group2endPose = (new Pose(119, 60, Math.toRadians(-10)));
    Pose goToOpenGatePose = (new Pose(127.4, 63, 0));
    Pose pickupFromGatePose = (new Pose(132, 59, Math.toRadians(30)));
    Pose group1endPose = (new Pose(119, 84, 0));
    Pose group3startPose = (new Pose(103, 42, Math.toRadians(-5)));
    Pose group3endPose = (new Pose(119, 36, Math.toRadians(-15)));
    Pose parkPose = (new Pose(97, 76, 0));

    public CmdCloseZoneAuto(Robot robot, Auto.Team team, Auto.AutoStrategy autoStrategy, double waitTime) {
        super(robot, team, autoStrategy, waitTime);

        startingPose = Auto.convertAlliancePose(startingPose, team);
        shootingPose = Auto.convertAlliancePose(shootingPose, team);
        group2startPose = Auto.convertAlliancePose(group2startPose, team);
        group2endPose = Auto.convertAlliancePose(group2endPose, team);
        goToOpenGatePose = Auto.convertAlliancePose(goToOpenGatePose, team);
        pickupFromGatePose = Auto.convertAlliancePose(pickupFromGatePose, team);
        group1endPose = Auto.convertAlliancePose(group1endPose, team);
        group3startPose = Auto.convertAlliancePose(group3startPose, team);
        group3endPose = Auto.convertAlliancePose(group3endPose, team);
        parkPose = Auto.convertAlliancePose(parkPose, team);

        robot.follower.setStartingPose(startingPose);
    }

    public void autonomousUpdate() {
        switch (autoState) {
            case START:
                if (waitTime <= 0 || timer.getElapsedTimeSeconds() > waitTime) {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }
                break;

            case DRIVE_TO_SHOOTING_SPOT:
                if (isFirstTimePath) {
                    robot.shooter.setVelocityTarget(AUTO_RPM);
                    robot.follower.followPath(getShootingPath(), true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    autoState = AutoState.SHOOTING;
                }
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
                    if (shotCount == 4) autoState = isCycleStrategy() ? AutoState.DRIVE_TO_GROUP : AutoState.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_GROUP:
                if (isFirstTimePath) {
                    robot.follower.followPath(getGroupDrivePath(), true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    autoState = AutoState.SLURPING_GROUP;
                }
                break;

            case SLURPING_GROUP:
                if (isFirstTimePath) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    robot.follower.followPath(getIntakePath(), intakePathSpeed, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy() || robot.intakeUptake.getNumberOfBallsStored() >= 3) {
                    isFirstTimePath = true;
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }
                break;

            case DRIVE_TO_PARK:
                if (isFirstTimePath) {
                    robot.follower.followPath(driveToPark, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.STOP;
                }
                break;

            case STOP:
                cancel();
                break;
        }
    }

    private PathChain getShootingPath() {
        if (shotCount == 0) return driveToShootPreloads;
        if (shotCount == 1) return driveToShootGroup2;
        if (shotCount == 2) return isCycleStrategy() ? driveToShootGateGroup : driveToShootGroup1;
        if (shotCount == 3) return isCycleStrategy() ? driveToShootGroup1 : driveToShootGroup3;
        return driveToShootGroup3; // shotCount == 4
    }

    private PathChain getGroupDrivePath() {
        if (shotCount == 1) return driveToGroup2;
        if (shotCount == 2) return isCycleStrategy() ? driveToOpenGate : driveToGroup1;
        if (shotCount == 3) return isCycleStrategy() ? driveToGroup1 : driveToGroup3;
        return driveToGroup3; // shotCount == 4
    }

    private PathChain getIntakePath() {
        if (shotCount == 1) return intakeGroup2;
        if (shotCount == 2) return isCycleStrategy() ? intakeFromGate : intakeGroup1;
        if (shotCount == 3) return isCycleStrategy() ? intakeGroup1 : intakeGroup3;
        return intakeGroup3; // shotCount == 4
    }

    @Override
    public void buildPaths() {
        driveToShootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group2startPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group2startPose.getHeading())
                .build();
        intakeGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group2startPose, group2endPose))
                .setLinearHeadingInterpolation(group2startPose.getHeading(), group2endPose.getHeading())
                .build();
        driveToShootGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group2endPose, shootingPose))
                .setLinearHeadingInterpolation(group2endPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToOpenGate = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, goToOpenGatePose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), goToOpenGatePose.getHeading())
                .build();
        intakeFromGate = robot.follower.pathBuilder()
                .addPath(new BezierLine(goToOpenGatePose, pickupFromGatePose))
                .setLinearHeadingInterpolation(goToOpenGatePose.getHeading(), pickupFromGatePose.getHeading())
                .build();
        driveToShootGateGroup = robot.follower.pathBuilder()
                .addPath(new BezierLine(pickupFromGatePose, shootingPose))
                .setLinearHeadingInterpolation(pickupFromGatePose.getHeading(), shootingPose.getHeading())
                .build();
        intakeGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group1endPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1endPose.getHeading())
                .build();
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group1endPose, shootingPose))
                .setLinearHeadingInterpolation(group1endPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group3startPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group3startPose.getHeading())
                .build();
        intakeGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group3startPose, group3endPose))
                .setLinearHeadingInterpolation(group3startPose.getHeading(), group3endPose.getHeading())
                .build();
        driveToShootGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group3endPose, shootingPose))
                .setLinearHeadingInterpolation(group3endPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
