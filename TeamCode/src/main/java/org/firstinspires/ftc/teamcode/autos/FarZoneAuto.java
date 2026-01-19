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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Zone Auto Red")
public class FarZoneAuto extends  OpMode{



    final double AUTO_RPM = 4000;
    enum AutoState {
        START,
        DRIVE_TO_SHOOTING_SPOT,
        SHOOTING,
        DRIVE_TO_GROUP,
        SLURPING_GROUP,
        DRIVE_TO_PARK, // <-- dunno if we need this but just in case
        DRIVE_TO_GATE, // <--- these are for later when we get more balls than the pre-placed ones
        SLURPING_FROM_GATE,
        STOP
    }

    Robot robot;
    Timer pathTimer;
    ShooterTask shooterTask;

    boolean isFirstTimePath = true;

    boolean editingAlliance = true;
    double intakePathSpeed = 0.6;
    int shotCount = 0;

    AutoState autoState = AutoState.DRIVE_TO_SHOOTING_SPOT;

    //Starting pose wrong
    PathChain driveToShootPreloads, driveToGroup1, driveToShootGroup1, driveToGroup2, driveToShootGroup2, driveToGroup3, driveToShootGroup3,
            intakeGroup1, intakeGroup2, intakeGroup3, driveToGate, driveToPark;

    Pose startingPose =(new Pose(120.5, 132, Math.toRadians(45)));
    Pose shootingPose = (new Pose(96, 96, Math.toRadians(45)));
    Pose group1startPose = (new Pose(99, 85, 0));
    Pose group2startPose =(new Pose(99, 61, 0) );
    Pose group3startPose = (new Pose(100, 35, 0) );
    Pose group1endPose =(new Pose(127.5, 85, 0) );
    Pose group2endPose = (new Pose(127, 61, 0) );
    Pose group3endPose = (new Pose(131, 35, 0) );
    Pose gatePose = (new Pose(120, 70, 0));
    Pose parkPose = (new Pose(120, 92, Math.PI*3/2));

    public void init() {

        robot = new Robot(hardwareMap, telemetry);
        shooterTask = new ShooterTask(robot);
        pathTimer = new Timer();


        robot.follower.setStartingPose(startingPose);
        robot.follower.setMaxPower(1);
        robot.shooter.setPitchDegrees(27.5);
        robot.shooter.setTurretDegrees(0.0);
        robot.intakeUptake.closeBlockingServo();
//        robot.shooter.setAlwaysAimShooter(false);
        buildPaths();
    }


    public void init_loop(){
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

    public void start(){
         startingPose = Robot.convertAlliancePose(startingPose);
         shootingPose = Robot.convertAlliancePose(shootingPose);
         group1startPose = Robot.convertAlliancePose(group1startPose);
         group2startPose = Robot.convertAlliancePose(group2startPose );
         group3startPose = Robot.convertAlliancePose(group3startPose);
         group1endPose = Robot.convertAlliancePose(group1endPose );
         group2endPose = Robot.convertAlliancePose(group2endPose);
         group3endPose = Robot.convertAlliancePose(group3endPose);
         gatePose = Robot.convertAlliancePose(gatePose);
         parkPose = Robot.convertAlliancePose(parkPose);


    }

    public void stop(){
        Robot.setTeleOpStartPose(robot.follower.getPose());
    }


//TODO: CHANGE
    public void loop(){
        autonomousUpdate();
            telemetry.addData("Current Auto State", autoState);
            telemetry.addData("Current Shooter State", shooterTask.getShooterState());
            telemetry.addData("follower busy?", robot.follower.isBusy());
            telemetry.addData("shooter velocity", robot.shooter.getVelocityRPM());
        telemetry.addData("Is Flywheel on target: ", robot.shooter.isFlywheelOnTarget(Shooter.Params.SHOOTER_TOLERANCE_RPM) + ", Is pitch on target: " + robot.shooter.isPitchOnTarget(Shooter.Params.PITCH_TOLERANCE));
        Robot.setTeleOpStartPose(robot.follower.getPose());
        telemetry.update();

    }



    public void autonomousUpdate() {
        switch (autoState) {
            case START:
                //intentionally fall through
            case DRIVE_TO_SHOOTING_SPOT:

                shooterTask.revUpShooterMotor(AUTO_RPM);

                if (shotCount == 0 && isFirstTimePath )
                {
                    robot.follower.followPath(driveToShootPreloads, true);
                    isFirstTimePath = false;
                }
                if (shotCount == 1 && isFirstTimePath) { robot.follower.followPath(driveToShootGroup1, true); isFirstTimePath = false;}
                if (shotCount == 2 && isFirstTimePath) {robot.follower.followPath(driveToShootGroup2, true); isFirstTimePath = false;}
                if (shotCount == 3 && isFirstTimePath) {robot.follower.followPath(driveToShootGroup3, true); isFirstTimePath = false;}
                if (!robot.follower.isBusy()) {
                    isFirstTimePath = true;
                    autoState = AutoState.SHOOTING;
                }

                break;
            case SHOOTING:

                if (isFirstTimePath){
                    shooterTask.startShooterTask();
                    isFirstTimePath = false;
                }

                if (shooterTask.isFinished()) {
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
                break;
        }

        shooterTask.update(AUTO_RPM);
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
