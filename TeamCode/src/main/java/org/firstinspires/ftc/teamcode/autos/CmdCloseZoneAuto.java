package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class CmdCloseZoneAuto extends AutoCommands {

    /*
    Shotcount Key:
    0 = preload
    1 = second row
    2 = gate pose/intaking
    3 = third row
    4 = first row

     */

    private static final double PICKUP_TIMEOUT = 5; // seconds
    PathChain driveToShootPreloads, driveToShootGroup1, driveToGroup1, driveToGroup2, driveToShootGroup2,driveToShootGate,
            driveToGroup3, driveToShootGroup3, intakeGroup1, intakeGroup2, intakeGroup3, driveToPark, driveToGate;

    Pose startingPose = (new Pose(114, 123, Math.toRadians(42)));
    Pose shootingPose = (new Pose(84 * autoScale, 85 * autoScale, Math.toRadians(37)));
    Pose group2startPose = (new Pose(99 * autoScale, 59 * autoScale, 0));
    Pose group2endPose = (new Pose(124 * autoScale, 51 * autoScale, 0));
    Pose group2ControlPose = new Pose(78 * autoScale, 59 * autoScale);
    Pose group1endPose = (new Pose(125 * autoScale, 85 * autoScale, 0));
    Pose group1StartPose = (new Pose(100 * autoScale, 85 * autoScale, 0));
    Pose group1ControlPose = new Pose(78 * autoScale, 81 * autoScale);

    Pose gatePose = (new Pose(127.5 * autoScale, (56.161) * autoScale, Math.toRadians(34.77854-11.57375)));
    Pose group3startPose = (new Pose(94 * autoScale, 35 * autoScale, 0));
    Pose group3endPose = (new Pose(131 * autoScale, 32 * autoScale, 0));
    Pose group3ControlPose = new Pose(60 * autoScale, 25 * autoScale);
    Pose parkPose = (new Pose(115 * autoScale, 65 * autoScale, 0));
    Pose gateControlPose =  new Pose(105 * autoScale, 42 * autoScale);
    Pose returnGroup3ControlPose = new Pose(52 * autoScale, 67 * autoScale);


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
        robot.follower.setMaxPower(1);
        Shooter.alwaysAimTurret = false;
        robot.shooter.setTurretDegrees(0.0);
        robot.shooter.setPitchDegrees(34.0);

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
                    robot.shooter.setVelocityTarget(CLOSE_AUTO_RPM);
                    robot.follower.followPath(getShootingPath(), true);
                    isFirstTimePath = false;
                }

//                if (!robot.follower.isBusy()) {
                if (!robot.follower.isBusy()){
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    isFirstTimePath = true;
                    setAutoState(AutoState.SHOOTING);
                }
                break;
            case SHOOTING:
                if (isFirstTimePath && timer.getElapsedTimeSeconds() > .65) {
                    robot.shooterTask.startTask(null);
                    isFirstTimePath = false;
                }

                if (robot.shooterTask.isFinished() && !isFirstTimePath) {
                    isFirstTimePath = true;
                    shotCount++;
                    if (shotCount <= 4) setAutoState(AutoState.SLURPING_GROUP);
                    if (shotCount == 5) setAutoState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case DRIVE_TO_GROUP:
                if (isFirstTimePath) {
                    robot.follower.followPath(getIntakePath(), true);
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
                    //slower second path?
                    if (shotCount == 1){
                        robot.follower.followPath(getIntakePath(), 1, true);

                    } else {
                        robot.follower.followPath(getIntakePath(), 1, true);
                    }
                    isFirstTimePath = false;
                }

                if (shotCount != 2  && !robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                }

                //This is for when you are at the gate you need to wait until you get 3 balls or timeout
                if (shotCount == 2 && (robot.intakeUptake.getNumberOfBallsStored() >= 3 || timer.getElapsedTimeSeconds() >= PICKUP_TIMEOUT)) {
                    isFirstTimePath = true;
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
        if (shotCount == 1) return driveToShootGroup2;
        if (shotCount == 2) return driveToShootGate;
        if (shotCount == 3) return driveToShootGroup3;
        return driveToShootGroup1; //shotCount == 3
    }

    private PathChain getIntakePath() {
        if(shotCount == 0 || shotCount == 1) return intakeGroup2;
        if (shotCount == 2) return driveToGate;
        if (shotCount == 3) return intakeGroup3;
        if (shotCount == 4) return intakeGroup1;
        return intakeGroup1; // shotCount == 3
    }

//    private PathChain getIntakePath() {
//        if (shotCount == 0 || shotCount == 1) return dri;
//        if (shotCount == 2) return intakeGroup2;
//        return intakeGroup3; // shotCount == 3
//    }

    @Override
    public void buildPaths() {
        driveToShootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startingPose, shootingPose))
                .setLinearHeadingInterpolation(startingPose.getHeading(), shootingPose.getHeading())
                .build();
//        driveToGroup1 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootingPose, group1StartPose))
//                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1StartPose.getHeading())
//                .build();
        intakeGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, group1ControlPose, group1endPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group1endPose.getHeading())
                .build();
        driveToShootGroup1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group1endPose, shootingPose))
                .setLinearHeadingInterpolation(group1endPose.getHeading(), shootingPose.getHeading())
                .build();
//        driveToGroup2 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootingPose, group2startPose))
//                .setLinearHeadingInterpolation(shootingPose.getHeading(), group2startPose.getHeading())
//                .build();
        intakeGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, group2ControlPose, group2endPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group2endPose.getHeading())
                .build();
        driveToShootGroup2 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(group2endPose, group2ControlPose, shootingPose))
                .setLinearHeadingInterpolation(group2endPose.getHeading(), shootingPose.getHeading())
                .build();
//        driveToGroup3 = robot.follower.pathBuilder()
//                .addPath(new BezierLine(shootingPose, group3startPose))
//                .setLinearHeadingInterpolation(shootingPose.getHeading(), group3startPose.getHeading())
//                .build();
        intakeGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose,group3ControlPose, group3endPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), group3endPose.getHeading())
                .build();
        driveToShootGroup3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(group3endPose, shootingPose))
                .setLinearHeadingInterpolation(group3endPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose, gateControlPose, gatePose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), gatePose.getHeading())
                .build();
        driveToShootGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(gatePose, gateControlPose, shootingPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), shootingPose.getHeading())
                .build();
        driveToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }
}