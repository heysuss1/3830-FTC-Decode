package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;

@Autonomous (name = "Red Side Auto")
public class TwelveBallAuto extends OpMode {

    enum AutoState {
        START,
        DRIVE_TO_SHOOTING_SPOT,
        SHOOTING,
        DRIVE_TO_GROUP_1,
        SLURPING_GROUP_1,
        DRIVE_TO_GROUP_2,
        SLURPING_GROUP_2,
        DRIVE_TO_GROUP_3,
        SLURPING_GROUP_3,
        DRIVE_TO_PARK, // <-- dunno if we need this but just in case
        DRIVE_TO_GATE, // <--- these are for later when we get more balls than the pre-placed ones
        SLURPING_FROM_GATE,
        STOP
    }

    Robot robot;
    Timer pathTimer;
    ShooterTask shooterTask;

    boolean editingAlliance = true;
    double intakePathSpeed = 0.6;
    int shotCount = 0;

    AutoState autoState = AutoState.START;

    //Starting pose wrong
    PathChain driveToShootPreloads, driveToGroup1, driveToShootGroup1, driveToGroup2, driveToShootGroup2, driveToGroup3, driveToShootGroup3,
            intakeGroup1, intakeGroup2, intakeGroup3, driveToGate, driveToPark;
    
    Pose startingPose = Robot.convertAlliancePose(new Pose(128, 118, Math.toRadians(40)));
    Pose shootingPose = Robot.convertAlliancePose(new Pose(96, 96, Math.toRadians(40)));
    Pose group1startPose = Robot.convertAlliancePose(new Pose(99, 83, 0));
    Pose group2startPose = Robot.convertAlliancePose(new Pose(99, 61, 0) );
    Pose group3startPose = Robot.convertAlliancePose(new Pose(100, 35, 0) );
    Pose group1endPose = Robot.convertAlliancePose(new Pose(127.5, 83, 0) );
    Pose group2endPose = Robot.convertAlliancePose(new Pose(124, 61, 0) );
    Pose group3endPose = Robot.convertAlliancePose(new Pose(131, 35, 0) );
    Pose gatePose = Robot.convertAlliancePose(new Pose(120, 70, 0));
    Pose parkPose = Robot.convertAlliancePose(new Pose(120, 92, Math.PI*3/2));

    public void init() {

        robot = new Robot(hardwareMap, telemetry);
        shooterTask = new ShooterTask(robot);
        pathTimer = new Timer();

        while (editingAlliance){
            if (gamepad1.dpad_up) {
                Robot.setTeam(Robot.Team.BLUE);
            }
            if (gamepad1.dpad_down){
                Robot.setTeam(Robot.Team.RED);
            }
            if (gamepad1.left_bumper){
                editingAlliance = false;
            }
            telemetry.addData("Team", Robot.getTEAM());
            telemetry.update();
        }

        robot.follower.setStartingPose(startingPose);
        robot.follower.setMaxPower(1);
        buildPaths();
    }

    public void loop(){
        autonomousUpdate();

        if (!Robot.inComp) {
            telemetry.addData("Current Auto State", autoState);
            telemetry.addData("Current Shooter State", shooterTask.getShooterState());
            telemetry.addData("follower busy?", robot.follower.isBusy());
            telemetry.addData("shooter velocity", robot.shooter.getVelocityRPM());
            telemetry.update();
        }
    }

    public void autonomousUpdate() {
        switch (autoState) {
            case START:
                //intentionally fall through
            case DRIVE_TO_SHOOTING_SPOT:

                if (shotCount == 0) robot.follower.followPath(driveToShootPreloads, true);
                if (shotCount == 1) robot.follower.followPath(driveToShootGroup1, true);
                if (shotCount == 2) robot.follower.followPath(driveToShootGroup2, true);
                if (shotCount == 3) robot.follower.followPath(driveToShootGroup3, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.SHOOTING;
                }

                break;
            case SHOOTING:

                shooterTask.startShooterTask();

                if (shooterTask.isFinished()) { //maybe this will be broken!
                    //TODO: fix this to be right idk
                    shotCount++;
                    if (shotCount == 1) autoState = AutoState.DRIVE_TO_GROUP_1;
                    if (shotCount == 2) autoState = AutoState.DRIVE_TO_GROUP_2;
                    if (shotCount == 3) autoState = AutoState.DRIVE_TO_GROUP_3;
                    if (shotCount == 4) autoState = AutoState.DRIVE_TO_PARK;
                }

                break;
            case DRIVE_TO_GROUP_1:

                robot.follower.followPath(driveToGroup1, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.SLURPING_GROUP_1;
                }

                break;
            case SLURPING_GROUP_1:

                robot.follower.followPath(intakeGroup1, intakePathSpeed, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }

                break;
            case DRIVE_TO_GROUP_2:

                robot.follower.followPath(driveToGroup2, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.SLURPING_GROUP_2;
                }

                break;
            case SLURPING_GROUP_2:

                robot.follower.followPath(intakeGroup2, intakePathSpeed, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }

                break;
            case DRIVE_TO_GROUP_3:

                robot.follower.followPath(driveToGroup3, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.SLURPING_GROUP_3;
                }

                break;
            case SLURPING_GROUP_3:

                robot.follower.followPath(intakeGroup3, intakePathSpeed, true);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;
                }

                break;
            case DRIVE_TO_PARK:

                robot.follower.followPath(driveToPark);

                if (!robot.follower.isBusy()) {
                    autoState = AutoState.STOP;
                }

                break;

                /*TODO: make the separate DRIVE_TO_GROUP_n's and SLURPING_GROUP_n's into their own single states*/
        }

        shooterTask.update();
        robot.shooter.shooterTask();
        robot.intakeUptake.intakeUptakeTask();
        robot.follower.update();
    }

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
