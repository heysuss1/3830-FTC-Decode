package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Tasks.Tasks;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.subsystems.Alignment;

@Config
@TeleOp (name = "TeleOp")
public class MainTele extends LinearOpMode {

    /*
    TODO: right trigger - shoot
    TODO: left trigger - intake  (and/or left trigger)
    TODO: left trigger - aim
    TODO: square - open gate

    */
    Hardware robot = Hardware.getInstance();
    Follower follower;
    Tasks tasks;
    Timer shooterTimer;

    VelocityController velController;
    RobotConstants constants = new RobotConstants();
    Transfer.Transfer_state transferState;
    RobotConstants.SystemState systemState;
    double batteryVoltage;
    int shotCounter = 0;
    int targetVel = 3600;
    boolean hadBall, hasBall;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(142, 54, Math.PI));
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(0.8);
        velController = new VelocityController(hardwareMap);
        batteryVoltage = velController.getBatteryVoltage();
        boolean intakeOn = false;
        boolean orienting = false;

        systemState = RobotConstants.SystemState.OFF;
        transferState = Transfer.Transfer_state.OFF;

        tasks = new Tasks(robot);

        waitForStart();
        shooterTimer = new Timer();

        while (opModeIsActive()) {
            robot.shooter.setRobotPose(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
            hadBall = hasBall;
            hasBall = robot.shooter.hasBall();

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, follower, orienting);

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
                robot.transfer.setOuttakeMode();
            }

            orienting = currentGamepad1.b;


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

            robot.shooter.setVelocity(targetVel);
            tasks.turretUpdate();
            tasks.updateShooter();
            tasks.updateTransfer(hasBall);

            telemetry.addData("Shooter vel: ", robot.shooter.getVelocity());
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("x: ", follower.getPose().getX());
            telemetry.addData("y: ", follower.getPose().getY());
            telemetry.addData("Heading: ", follower.getHeading());
            telemetry.addData("Shooter Timer", shooterTimer.getElapsedTimeSeconds());
            telemetry.addData("Shooter state", systemState);
            telemetry.addData("yawAligned?: ", Alignment.yawAligned(follower.getPose().getX(), follower.getPose().getY(), 12, 140.4, follower.getHeading()));
            telemetry.update();
            follower.update();


        }
    }
}