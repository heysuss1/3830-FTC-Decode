package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

    //TODO: a button to relocalize robot in our human player zone.
    /*
    cross - shoot
    right bumper - intake on and off
    right trigger - slow mode
    circle - outtake
    */

    Robot robot;
    Timer loopTimer;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    final double MAX_SPEED = 0.8;
    double currentTime = 0, lastTime = 0;
    boolean intakeOn = false;
    boolean orienting = false;
    int startingPoseIndex = 0;

    private boolean lockTurret = true;
    private boolean alwaysSetVelocity = true;
    private boolean alwaysSetPitch = true;

    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        loopTimer = new Timer();

        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(MAX_SPEED);

        robot.shooter.setAlwaysAimTurret(lockTurret);
        robot.shooter.setAlwaysAimPitch(alwaysSetPitch);
        robot.shooter.setAlwaysSetVelocity(alwaysSetVelocity);

        while (opModeInInit()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                startingPoseIndex = Math.floorMod(startingPoseIndex + 1, 4);
            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                startingPoseIndex = Math.floorMod(startingPoseIndex + 1, 4);

            }
            telemetry.addData("Starting Pose", Robot.POSE_NAME_LIST[startingPoseIndex]);
            telemetry.update();
        }

        if (Robot.getTeleOpStartPose() == null) {
            robot.follower.setStartingPose(Robot.POSE_LIST[startingPoseIndex]);
        } else {
            robot.follower.setStartingPose(Robot.getTeleOpStartPose());
        }

        waitForStart();

        while (opModeIsActive()) {

            //Tracks loop times
            lastTime = currentTime;
            currentTime = loopTimer.getElapsedTime();

            //Copying our game
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, robot.follower, orienting);

            //When right bumper is pressed, intake toggles between on and off states.
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeOn = !intakeOn;  // Toggle the boolean
                robot.intakeUptake.setIntakeUptakeMode(
                        intakeOn ? IntakeUptake.intakeUptakeStates.INTAKING
                                : IntakeUptake.intakeUptakeStates.OFF
                );
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                lockTurret = !lockTurret;  // Toggle the boolean
                robot.shooter.setAlwaysAimTurret(lockTurret);
                if (!lockTurret){
                    robot.shooter.setTurretDegrees(0.0);
                }
            }

            if (currentGamepad1.back && !previousGamepad1.back){
                alwaysSetVelocity = !alwaysSetVelocity;
                robot.shooter.setAlwaysSetVelocity(alwaysSetVelocity);
                if (!alwaysSetVelocity){
                    robot.shooter.setVelocityTarget(3750.0);
                    robot.shooter.setPitchDegrees(27.0);
                }
            }

            if (currentGamepad1.circle && !previousGamepad1.circle) {
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OUTTAKING);
                intakeOn = false;
            }

            if (currentGamepad1.right_trigger > 0.1)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(MAX_SPEED);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                robot.shooterTask.startShooterTask();
            }

            if (currentGamepad1.cross && !previousGamepad1.cross) {
                robot.shooterTask.cancelShooterUpdate();
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                intakeOn = false;
            }

            if (robot.isInRevUpZone()){
                robot.shooterTask.revUpShooterMotor(robot.shooter.getVelocityTarget());
            }

            robot.shooterTask.update();
            robot.follower.update();
            robot.shooter.shooterTask();
            robot.intakeUptake.intakeUptakeTask();

            telemetry.addData("Shooter vel: ", robot.shooter.getVelocityRPM());
            telemetry.addData("Loop Time", currentTime - lastTime);
            telemetry.addData("x: ", robot.follower.getPose().getX());
            telemetry.addData("y: ", robot.follower.getPose().getY());
            telemetry.addData("Heading: ", robot.follower.getHeading());
            telemetry.addData("Shooter param rpm", robot.shooter.getVelocityTarget());
            telemetry.addData("Current Turret Degrees", robot.shooter.getTurretDegrees());
            telemetry.addData("Color Sensor 1 Distance", robot.intakeUptake.getColorSensor1Distance());
            telemetry.addData("Color Sensor 2 Distance", robot.intakeUptake.getColorSensor2Distance());
            telemetry.addData("Color Sensor 3 Distance", robot.intakeUptake.getColorSensor3Distance());
            telemetry.addData("Number of big BLACK balls", robot.intakeUptake.getNumberOfBallsStored());
            telemetry.update();
        }
    }
}
