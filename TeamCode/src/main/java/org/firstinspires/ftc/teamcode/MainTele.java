package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp (name = "TeleOp")
public class MainTele extends LinearOpMode {

    /*
    TODO: right trigger - shoot
    TODO: left trigger - intake  (and/or left trigger)
    TODO: left trigger - aim
    TODO: square - open gate

     */
    Hardware robot = Hardware.getInstance();
    int shooterState;
    Timer uptakeTimer;

    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(1);
        boolean intakeOn = false;
        boolean rightBumper = false;
        shooterState = -1;
        waitForStart();
        uptakeTimer = new Timer();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1);
            /*
            When the right bumper is clicked, if the intake is not on start the intake. If it is on, stop the intake.
//             */
            if (currentGamepad1.left_trigger > 0.1){
                robot.intake.startIntake();
                robot.ramp.startRamp();//wouldnt that start the intake over and over?
            } else{
                robot.intake.stopIntake();
                robot.ramp.stopRamp();
            }

            if (currentGamepad1.right_trigger > 0.1)
                robot.driveTrain.setSpeed(0.3);
            else {
                robot.driveTrain.setSpeed(1);
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
//                resetTimer(robot.shooter.uptakeTimer);
//                robot.shooter.startUptake(1);
//                //TODO: add Shoot Method
                setShooterState(0);
            }


            shooterUpdate();
            telemetry.update();


        }
    }

    public void shooterUpdate(){
        switch (shooterState){
            case 0:
                //Set power to needed velocity.
                if (uptakeTimer.getElapsedTimeSeconds() > 2){ //TODO: make better time!!!!!
                    setShooterState(1);
                }
                break;
            //                robot.shooter.startUptake();
                if (uptakeTimer.getElapsedTimeSeconds() > 1.2){
                    robot.shooter.stopUptake();
                    setShooterState(1);
                }
                break;
            case 1:
                robot.shooter.startUptake();
                if (uptakeTimer.getElapsedTimeSeconds() > 0.5){
                    robot.shooter.stopUptake();

                }
                break;

        }
    }
    public void setShooterState(int state){
        shooterState = state;
        uptakeTimer.resetTimer();
    }
}
