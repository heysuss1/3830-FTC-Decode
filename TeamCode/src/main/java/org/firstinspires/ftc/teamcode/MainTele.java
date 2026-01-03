package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tasks.Tasks;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.controllers.VelocityController;

@Config
@TeleOp (name = "TeleOp")
public class MainTele extends LinearOpMode {

    /*
    TODO: right trigger - shoot
    TODO: left trigger - intake  (and/or left trigger)
    TODO: left trigger - aim
    TODO: square - open gate

    */
    Hardware robot;
    Tasks tasks;
    VelocityController velController;
    RobotConstants constants = new RobotConstants();
    Timer loopTimer;
    double currentTime = 0, lastTime = 0;
    int targetVel = 3600;
    boolean hadBall, hasBall, showTelemetry;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    int loopCount = 0;


    public void runOpMode() {
        robot = new Hardware(hardwareMap, telemetry);
        robot.follower.setStartingPose(new Pose(142, 54, Math.PI));
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(0.8);
        velController = new VelocityController(hardwareMap);
        boolean intakeOn = false;
        boolean orienting = false;
        boolean showTelemetry = false;

        tasks = new Tasks(robot, hardwareMap, false);

        loopTimer = new Timer();

        waitForStart();
        while (opModeIsActive()) {

//            for (LynxModule hub: hardwareMap.getAll(LynxModule.class)){
//                hub.clearBulkCache();
//            }
            lastTime = currentTime;
            currentTime = loopTimer.getElapsedTime();
            hadBall = hasBall;
            hasBall = robot.shooter.hasBall();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, robot.follower, orienting);

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (!intakeOn) {
                    tasks.setTransferState(Tasks.TransferState.INTAKE);
                    intakeOn = true;
                } else {
                    tasks.setTransferState(Tasks.TransferState.OFF);
                    intakeOn = false;
                }
            }
            if (currentGamepad1.y) {
                robot.transfer.startFeed();
            }

            if (currentGamepad1.circle) {
                tasks.setTransferState(Tasks.TransferState.OUTTAKE);
            }

            if (currentGamepad1.right_trigger > 0.1 || currentGamepad1.x)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(1);
            }

//            if (currentGamepad1.x) {
//                if (Alignment.yawAligned(follower.getPose().getX(), follower.getPose().getY(), 142,140.4, follower.getHeading())) {
//                        systemState = RobotConstants.SystemState.SPEEDING_UP;
//                } else {
//                    Alignment.alignYaw(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading(), 142, 140.4);
//                }
//            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                tasks.setShooterState(Tasks.ShooterState.SPEEDING_UP);
            }
            if (currentGamepad1.cross) {
                tasks.setShooterState(Tasks.ShooterState.DONE);
                tasks.setTransferState(Tasks.TransferState.OFF);
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                targetVel += 100;
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                targetVel -= 100;
            }

            robot.shooter.setVelocityTarget(targetVel);
            tasks.update(hasBall, hadBall);


            if (showTelemetry){

            }



            robot.follower.update();

            if (loopCount % 5 == 0){
                telemetry.addData("Shooter vel: ", robot.shooter.getVelocity());
                telemetry.addData("Loop Time", currentTime - lastTime);
                telemetry.addData("Target Vel", targetVel);
                telemetry.addData("x: ", robot.follower.getPose().getX());
                telemetry.addData("y: ", robot.follower.getPose().getY());
                telemetry.addData("Heading: ", robot.follower.getHeading());
                telemetry.update();
            }
            loopCount++;
        }
    }
}