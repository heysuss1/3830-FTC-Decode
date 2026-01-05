package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.tasks.Tasks;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

    /*
    cross - shoot
    right bumper - intake on and off
    right trigger - slow mode
    circle - outtake
    */

    Robot robot;
    Tasks tasks;
    Timer loopTimer;
    double currentTime = 0, lastTime = 0;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    final double MAX_SPEED = 0.8;
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);

        //I did this because the robots positions should be stored after auto, and it would only need to be manually
        //set if doing driver practice and thus not in comp.
        if (!Robot.inComp){
            robot.follower.setStartingPose(new Pose(142, 54, Math.PI));
        }
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(MAX_SPEED);

        boolean intakeOn = false;
        boolean orienting = false;

        tasks = new Tasks(robot);

        loopTimer = new Timer();

        waitForStart();
        while (opModeIsActive()) {
            lastTime = currentTime;
            currentTime = loopTimer.getElapsedTime();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, robot.follower, orienting);

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeOn = !intakeOn;  // Toggle the boolean
                robot.intakeUptake.setIntakeUptakeMode(
                        intakeOn ? IntakeUptake.intakeUptakeStates.INTAKING
                                : IntakeUptake.intakeUptakeStates.OFF
                );
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
                tasks.setShooterState(Tasks.ShooterState.OPEN_BLOCKING_SERVO);
            }

            if (currentGamepad1.cross && !previousGamepad1.cross) {
                tasks.cancelShooterUpdate();
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                intakeOn = false;
            }

            tasks.update(hasBall, hadBall);
            robot.follower.update();
            robot.shooter.shooterTask();
            robot.intakeUptake.intakeUptakeTask();

            if (!Robot.inComp){
                telemetry.addData("Shooter vel: ", robot.shooter.getVelocityRPM());
                telemetry.addData("Loop Time", currentTime - lastTime);
                telemetry.addData("x: ", robot.follower.getPose().getX());
                telemetry.addData("y: ", robot.follower.getPose().getY());
                telemetry.addData("Heading: ", robot.follower.getHeading());
                telemetry.update();
            }
        }
    }
}