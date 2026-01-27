package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;

public class CmdFarZoneAuto extends AutoCommands {

    PathChain driveToIntakeHP1, driveToShootForPickup,
            driveToIntakeRow3, driveToShootRow3, driveToIntakeHP2, huntPath1, huntPath2,
            huntPath3, driveToShootGateBalls, driveToPark;

    PathChain driveToShootGroup1;
    Pose startPose = new Pose(103, 8.1, 0);
    Pose intakeHumanPlayerPose = new Pose(134, 8.5, 0);
    Pose shootIntakeRowPose = new Pose(87, 22, Math.toDegrees(20));
    Pose intakeRow3Pose = new Pose(119, 36, Math.toDegrees(0));
    Pose shootingPose = new Pose(100, 9, 0);
    Pose intakeGateBallsPath1 = new Pose (126, 12, Math.toDegrees(30));
    Pose intakeGateBallsPath2 = new Pose (133, 16, Math.toDegrees(70));
    Pose intakeGateBallsPath3 = new Pose (135, 39, Math.toDegrees(90));
    Pose parkPose = new Pose(106, 10,0);

    int huntPath = 1;

    public CmdFarZoneAuto(Robot robot, Auto.Team team, Auto.AutoStrategy autoStrategy,double waitTime){
        super(robot, team, autoStrategy, waitTime);

        startPose = Auto.convertAlliancePose(startPose, team);
        intakeHumanPlayerPose = Auto.convertAlliancePose(intakeHumanPlayerPose, team);
        shootIntakeRowPose = Auto.convertAlliancePose(shootIntakeRowPose, team);
        intakeRow3Pose = Auto.convertAlliancePose(intakeRow3Pose, team);
        shootingPose = Auto.convertAlliancePose(shootingPose, team);
        intakeGateBallsPath1 = Auto.convertAlliancePose(intakeGateBallsPath1, team);
        intakeGateBallsPath2 = Auto.convertAlliancePose(intakeGateBallsPath2, team);
        intakeGateBallsPath3 = Auto.convertAlliancePose(intakeGateBallsPath3, team);
        parkPose = Auto.convertAlliancePose(parkPose, team);

        robot.follower.setPose(startPose);
    }

    @Override
    public void autonomousUpdate(){
        switch (autoState) {
            case START:
                if(isFirstTimePath) robot.shooter.setVelocityTarget(AUTO_RPM);
                isFirstTimePath = false;
                if (waitTime <= 0 || timer.getElapsedTimeSeconds() > waitTime) {
                    setAutoState(AutoState.SHOOTING);
                }
                break;

            case DRIVE_TO_SHOOTING_SPOT:
                if (isFirstTimePath) {
                    robot.shooter.setVelocityTarget(AUTO_RPM);
                    if(shotCount == 1) robot.follower.followPath(driveToShootRow3, true);
                    if(shotCount == 2) robot.follower.followPath(driveToShootGateBalls, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    setAutoState(AutoState.SHOOTING);
                }

            case SHOOTING:
                if (isFirstTimePath) {
                    robot.shooterTask.startTask(null);
                    isFirstTimePath = false;
                }

                if (robot.shooterTask.isFinished()) {
                    isFirstTimePath = true;
                    shotCount++;
                    if (shotCount == 1) setAutoState(AutoState.SLURPING_GROUP);
                    if (shotCount == 2 && isCycleStrategy()) setAutoState(AutoState.SLURPING_GROUP);
                    else setAutoState(AutoState.DRIVE_TO_PARK);
                }
                break;

            case SLURPING_GROUP:

                if (isFirstTimePath) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    if(shotCount == 1) robot.follower.followPath(driveToIntakeHP1, true);
                    if(shotCount == 2) robot.follower.followPath(driveToIntakeHP2, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    if(shotCount == 2 && isCycleStrategy()) {
                        setAutoState(AutoState.HUNT_FOR_BALLS);
                    } else {
                        setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                        robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    }
                }

                break;

            case HUNT_FOR_BALLS:
                if (isFirstTimePath && huntPath == 1) {
                    robot.follower.followPath(huntPath1, true);
                    isFirstTimePath = false;
                }
                else if (isFirstTimePath && huntPath == 2) {
                    robot.follower.followPath(huntPath2, true);
                    isFirstTimePath = false;
                }
                else if (isFirstTimePath && huntPath == 3) {
                    robot.follower.followPath(huntPath3, true);
                    isFirstTimePath = false;
                }

                if (!robot.follower.isBusy() && huntPath < 3) {
                    isFirstTimePath = true;
                    huntPath++;
                }

                if (robot.intakeUptake.getNumberOfBallsStored() >= 3)
                {
                    isFirstTimePath = true;
                    huntPath = 1;
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
    public void buildPaths() {
        driveToIntakeHP1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, intakeHumanPlayerPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakeHumanPlayerPose.getHeading())
                .build();
        driveToShootForPickup = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeHumanPlayerPose, shootIntakeRowPose))
                .setLinearHeadingInterpolation(intakeHumanPlayerPose.getHeading(), shootIntakeRowPose.getHeading())
                .build();
        driveToIntakeRow3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootIntakeRowPose, intakeRow3Pose))
                .setLinearHeadingInterpolation(shootIntakeRowPose.getHeading(), intakeRow3Pose.getHeading())
                .build();
        driveToShootRow3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeRow3Pose, shootingPose))
                .setLinearHeadingInterpolation(intakeRow3Pose.getHeading(), shootingPose.getHeading())
                .build();
        driveToIntakeHP2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, intakeHumanPlayerPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), intakeHumanPlayerPose.getHeading())
                .build();
        huntPath1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeHumanPlayerPose, intakeGateBallsPath1))
                .setLinearHeadingInterpolation(intakeHumanPlayerPose.getHeading(), intakeGateBallsPath1.getHeading())
                .build();
        huntPath2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeGateBallsPath1, intakeGateBallsPath2))
                .setLinearHeadingInterpolation(intakeGateBallsPath1.getHeading(), intakeGateBallsPath2.getHeading())
                .build();
        huntPath3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeGateBallsPath2, intakeGateBallsPath3))
                .setLinearHeadingInterpolation(intakeGateBallsPath2.getHeading(), intakeGateBallsPath3.getHeading())
                .build();
        driveToShootGateBalls = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeGateBallsPath3, shootingPose))
                .setLinearHeadingInterpolation(intakeGateBallsPath3.getHeading(), shootingPose.getHeading())
                .build();
        driveToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
