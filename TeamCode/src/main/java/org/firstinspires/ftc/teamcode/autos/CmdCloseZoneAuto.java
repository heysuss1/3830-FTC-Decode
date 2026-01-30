package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class CmdCloseZoneAuto extends AutoCommands {

    private static final double PICKUP_TIMEOUT = 3.5; // seconds
    PathChain driveToShootPreloads, driveToShootGroup1, driveToGroup1, driveToGroup2, driveToShootGroup2,
            driveToGroup3, driveToShootGroup3, intakeGroup1, intakeGroup2, intakeGroup3, driveToPark;

    Pose startingPose = (new Pose(114, 123, Math.toRadians(42)));
    Pose shootingPose = (new Pose(84 * autoScale, 85 * autoScale, Math.toRadians(42)));
    Pose group2startPose = (new Pose(99 * autoScale, 61 * autoScale, 0));
    Pose group2endPose = (new Pose(127 * autoScale, 61 * autoScale, 0));
    Pose group1endPose = (new Pose(127.5 * autoScale, 85 * autoScale, 0));
    Pose group1StartPose = (new Pose(100 * autoScale, 85 * autoScale, 0));
    ;
    Pose group3startPose = (new Pose(100 * autoScale, 35 * autoScale, 0));
    Pose group3endPose = (new Pose(131 * autoScale, 35 * autoScale, 0));
    Pose parkPose = (new Pose(97 * autoScale, 76 * autoScale, 0));

    public CmdCloseZoneAuto(Robot robot, Auto.Team team, Auto.AutoStrategy autoStrategy, double waitTime) {
        super(robot, team, autoStrategy, waitTime);

        startingPose = Auto.convertAlliancePose(startingPose, team);
        shootingPose = Auto.convertAlliancePose(shootingPose, team);
        group2startPose = Auto.convertAlliancePose(group2startPose, team);
        group2endPose = Auto.convertAlliancePose(group2endPose, team);
        group1endPose = Auto.convertAlliancePose(group1endPose, team);
        group3startPose = Auto.convertAlliancePose(group3startPose, team);
        group3endPose = Auto.convertAlliancePose(group3endPose, team);
        parkPose = Auto.convertAlliancePose(parkPose, team);

        robot.follower.setStartingPose(startingPose);
        robot.follower.setMaxPower(.8);
        Shooter.alwaysAimTurret = false;
        robot.shooter.setTurretDegrees(0.0);
        robot.shooter.setPitchDegrees(33.0);

    }

    public void autonomousUpdate() {
        switch (autoState) {
            case START:
                if (waitTime <= 0 || timer.getElapsedTimeSeconds() > waitTime) {
                    shotCount = 0;
                    setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                }
                break;

            case DRIVE_TO_SHOOTING_SPOT:
                if (isFirstTimePath) {
                    robot.shooter.setVelocityTarget(AUTO_RPM);
                    robot.follower.followPath(getShootingPath(), true);
                    isFirstTimePath = false;
                }

//                if (!robot.follower.isBusy()) {
                if (robot.follower.atParametricEnd()){
                    isFirstTimePath = true;
                    setAutoState(AutoState.SHOOTING);
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
                    if (shotCount == 1) setAutoState(AutoState.SLURPING_GROUP);
                    if (shotCount <= 3) setAutoState(AutoState.DRIVE_TO_GROUP);
                    if (shotCount == 4) setAutoState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_GROUP:
                if (isFirstTimePath) {
                    robot.follower.followPath(getGroupDrivePath(), true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    setAutoState(AutoState.SLURPING_GROUP);
                }
                break;

            case SLURPING_GROUP:
                if (isFirstTimePath) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    robot.follower.followPath(getIntakePath(), intakePathSpeed, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                }
                break;

            case DRIVE_TO_PARK:
                if (isFirstTimePath) {
                    robot.follower.followPath(driveToPark, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    setAutoState(AutoState.STOP);
                }
                break;

            case STOP:
                cancel();
                break;
        }
    }

    private PathChain getShootingPath() {
        if (shotCount == 0) return driveToShootPreloads;
        if (shotCount == 1) return driveToShootGroup1;
        if (shotCount == 2) return driveToShootGroup2;
        return driveToShootGroup3; //shotCount == 3
    }

    private PathChain getGroupDrivePath() {
        if(shotCount == 0 || shotCount == 1) return driveToGroup1;
        if (shotCount == 2) return driveToGroup2;
        return driveToGroup3; // shotCount == 3
    }

    private PathChain getIntakePath() {
        if (shotCount == 1) return intakeGroup1;
        if (shotCount == 2) return intakeGroup2;
        return intakeGroup3; // shotCount == 3
    }

    @Override
    public void buildPaths() {
        driveToShootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group1StartPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1StartPose.getHeading())
                .build();
        intakeGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, group1endPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1endPose.getHeading())
                .build();
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group1endPose, shootingPose))
                .setLinearHeadingInterpolation(group1endPose.getHeading(), shootingPose.getHeading())
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