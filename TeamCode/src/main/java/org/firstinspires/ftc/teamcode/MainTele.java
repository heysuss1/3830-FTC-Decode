package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

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
    int shooterState;
    Timer uptakeTimer;

    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(0.8);
        double batteryVoltage = 0;
        boolean intakeOn = false;
        String team = "RED";
        int startingState = 0;
        String[] startingStateList = {"Red Back", "Red Goal", "Blue Back", "Blue Goal"};
        double[] startXposes = {60, 120, 85, 24};
        double[] startYposes = {8, 128, 8, 128};
        double[] startHeadings = {0, 36, 0, -36};
        boolean orienting = false;
        boolean rightBumper = false;
        shooterState = -1;
        waitForStart();
        uptakeTimer = new Timer();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1, follower, orienting);
            /*
            When the right bumper is clicked, if the intake is not on start the intake. If it is on, stop the intake.
//             */
//            if (currentGamepad1.left_trigger > 0.1){
//                robot.intake.startIntake();
//                robot.ramp.startRamp();//wouldnt that start the intake over and over?
//            } else{
//                robot.intake.stopIntake();
//                robot.ramp.stopRamp();
//            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                if (!intakeOn){
                    robot.transfer.setIntakeMode();
                    intakeOn = true;
                }
                else{
                    robot.transfer.

                            stopIntake();
                    robot.transfer.stopRamp();
                    robot.transfer.stopFeed();
                    intakeOn = false;
                }
            }

            if (currentGamepad1.y) {
                robot.transfer.setFeedIntakeMode();
            }

            if (currentGamepad1.dpad_up) {
                team = "RED";
            }

            if (currentGamepad1.dpad_down) {
                team = "BLUE";
            }

//            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
//                startingState ++;
//                if (startingState > 4) {startingState = 0;}
//                follower.setPose(new Pose(startXposes[startingState], startYposes[startingState], startHeadings[startingState]));
//            }

            orienting = currentGamepad1.b;

            if (currentGamepad1.right_trigger > 0.1)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(1);
            }

            if (currentGamepad1.x) {
                shooterState = 0;
            }
            if (currentGamepad1.b){
                shooterState = 3;
            }

            for (VoltageSensor sensor: hardwareMap.voltageSensor){
                double voltage = sensor.getVoltage();;
                if (voltage > 0){
                    batteryVoltage = voltage;
                    break;
                }
            }
            shooterUpdate();
            telemetry.addData("Starting position", startingStateList[startingState]);
            telemetry.addData("Shooter vel: ", robot.shooter.getVelocity());
            telemetry.addData("Shooter vel (raw): ", robot.shooter.shootingMotor.getVelocity());
            telemetry.addData("Shooting state: ", shooterState);
            telemetry.addData("Shooter is ready to shoot: ", robot.shooter.isReady(3500, 40));
            telemetry.addData("Battery Voltage: ", batteryVoltage);
            telemetry.addData("Team", team);
            telemetry.update();


        }
    }

    public void shooterUpdate(){
        int tolerance;
        double batteryVoltage;
        switch (shooterState){
            case 0:
                //Set power to needed velocity.
                robot.transfer.stopRamp();
                robot.shooter.setVelocity(3500);
                if (robot.shooter.isReady(3350, 400)){
                    setShooterState(1);
                }
                break;
            case 15:
                robot.transfer.moveBackwards();
                if (uptakeTimer.getElapsedTimeSeconds() > 1){
                    setShooterState(1);
                }
               break;
            case 1:
                robot.transfer.setFeedMode();
                robot.transfer.startRamp();
                if (uptakeTimer.getElapsedTimeSeconds() > 4){
                    setShooterState(2);
                }
                break;
            case 2:
                robot.shooter.stopShooter();
                setShooterState(25);
                break;
            case 25:
                robot.transfer.stopFeed();
                robot.transfer.stopRamp();
                setShooterState(27);
                break;
            case 3:
                robot.shooter.prepShooter();
                break;

        }
    }
    public void setShooterState(int state){
        shooterState = state;
        uptakeTimer.resetTimer();
    }
}
