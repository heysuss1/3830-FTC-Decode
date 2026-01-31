package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;

public class CmdFarZoneAuto extends AutoCommands {

    public static final double TIME_BEFORE_NEXT_INTAKE_ATTEMPT = 5.0; // seconds


    boolean intakeTimerHasBeenReset = false;
    PathChain driveToPreload, driveToIntakeHP1, driveToShootFromHP,
            driveToIntakeRow3, driveToShootRow3, driveToIntakeHP2, huntPath1, huntPath2,
            huntPath3, driveToShootGateBalls, driveToPark;

    PathChain driveToShootGroup1;
    Pose startPose = new Pose(80, 9, Math.toRadians(90));
    Pose intakeHumanPlayerPose = new Pose(137.5, 16, 0);
    Pose shootIntakeRowPose = new Pose(87, 22, Math.toRadians(63));
    Pose intakeRow3Pose = new Pose(137.5, 42, Math.toRadians(0));
//    Pose shootingPose = new Pose(87, 22, Math.toRadians(61.5));  heading is zeroed;
    Pose shootingPose = new Pose(87, 22, Math.toRadians(61.5));
    Pose intakeRow3ControlPose = new Pose(63, 45);
    Pose intakeGateBallsPath1 = new Pose (126, 12, Math.toRadians(30));
    Pose intakeGateBallsPath2 = new Pose (133, 16, Math.toRadians(70));
    Pose intakeGateBallsPath3 = new Pose (135, 39, Math.toRadians(90));
    Pose parkPose = new Pose(113, 18,0);

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
        robot.shooter.setPitchDegrees(41.0);
        robot.shooter.setTurretDegrees(0.0);
    }

    @Override
    public void autonomousUpdate(){
        switch (autoState) {
            case START:
                if(isFirstTimePath) robot.shooter.setVelocityTarget(FAR_AUTO_RPM);
                isFirstTimePath = false;
                if (waitTime <= 0 || timer.getElapsedTimeSeconds() > waitTime) {
                    isFirstTimePath = true;
                    setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                }
                break;

            case DRIVE_TO_SHOOTING_SPOT:
                if (isFirstTimePath) {
                    intakeTimerHasBeenReset = false; // I do it again for redundancy js to be sure
                    robot.shooter.setVelocityTarget(FAR_AUTO_RPM);
                    if(shotCount == 0) robot.follower.followPath(driveToPreload, true);
                    if(shotCount == 1) robot.follower.followPath(driveToShootFromHP, true);
                    if(shotCount == 2) robot.follower.followPath(driveToShootRow3, true);
                    if(shotCount == 3) robot.follower.followPath(driveToShootFromHP, true);
                    isFirstTimePath = false;
                }

                if ((intakeTimerHasBeenReset && timer.getElapsedTimeSeconds() > .5)){
                    intakeTimerHasBeenReset = false;
                    isFirstTimePath = true;
                    setAutoState(AutoState.SHOOTING);
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                }

                if (!robot.follower.isBusy() && !intakeTimerHasBeenReset) {
                    timer.resetTimer();
                    intakeTimerHasBeenReset = true;

                }
                break;

            case SHOOTING:
                if (isFirstTimePath) {
                    robot.shooterTask.startTask(FAR_AUTO_RPM);
                    isFirstTimePath = false;
                }

                if (robot.shooterTask.isFinished()) {
                    isFirstTimePath = true;
                    shotCount++;
                    if (shotCount <=3) setAutoState(AutoState.SLURPING_GROUP);
                    else setAutoState(AutoState.DRIVE_TO_PARK);
                }
                break;
            case SLURPING_GROUP:

                if (isFirstTimePath) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
                    intakeTimerHasBeenReset = false;
                    if(shotCount <= 1) robot.follower.followPath(driveToIntakeHP1, true);
                    if(shotCount == 2) robot.follower.followPath(driveToIntakeRow3, .7, true);
                    if (shotCount == 3) robot.follower.followPath(driveToIntakeHP2, true);
                    isFirstTimePath = false;
                }

                //If we have been sitting at the human player zone for 1.25 seconds, or if we're intaking the 3rd row, go on to next state
                if ((intakeTimerHasBeenReset && timer.getElapsedTimeSeconds() > 1.4) || (shotCount == 2 && intakeTimerHasBeenReset)){
                    intakeTimerHasBeenReset = false;
                    isFirstTimePath = true;
                    setAutoState(AutoState.DRIVE_TO_SHOOTING_SPOT);
                }

                if (!robot.follower.isBusy() && !intakeTimerHasBeenReset) {
                    timer.resetTimer();
                    intakeTimerHasBeenReset = true;
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
         driveToPreload = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToIntakeHP1 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, intakeHumanPlayerPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), intakeHumanPlayerPose.getHeading())
                .build();
        driveToShootFromHP = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeHumanPlayerPose, shootingPose))
                .setLinearHeadingInterpolation(intakeHumanPlayerPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToIntakeRow3 = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootingPose,intakeRow3ControlPose, intakeRow3Pose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), intakeRow3Pose.getHeading())
                .build();
        driveToShootRow3 = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeRow3Pose,shootingPose))
                .setLinearHeadingInterpolation(intakeRow3Pose.getHeading(), shootingPose.getHeading())
                .build();
        driveToIntakeHP2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, intakeHumanPlayerPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), intakeHumanPlayerPose.getHeading())
                .build();
        driveToShootGateBalls = robot.follower.pathBuilder()
                .addPath(new BezierLine(intakeHumanPlayerPose, shootingPose))
                .setLinearHeadingInterpolation(intakeHumanPlayerPose.getHeading(), shootingPose.getHeading())
                .build();
        driveToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, parkPose))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), parkPose.getHeading())
                .build();
    }
}
