package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems.IntakeNFTC;

public class TeleOp extends LinearOpMode {

    /*
    TODO: right trigger - shooter
    TODO: left bumper - intake  (and/or left trigger)
    TODO: left trigger - aim
    TODO: square - open gate

     */
    Hardware robot = Hardware.getInstance();
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(1);
        boolean intakeOn = false;
        boolean slowMode = false;
        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            robot.driveTrain.moveRobot(currentGamepad1);

            /*
            When the right bumper is clicked, if the intake is not on start the intake. If it is on, stop the intake.
             */
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper){
                if (!intakeOn){
                    robot.intake.startIntake();
                    intakeOn = true;
                } else{
                    robot.intake.stopIntake();
                    intakeOn = false;
                }
            }

            if (currentGamepad1.x && !previousGamepad1.x){
                if (!slowMode){
                    robot.driveTrain.setSpeed(.3);
                    slowMode = true;
                } else{
                    robot.driveTrain.setSpeed(1);
                    slowMode = false;
                }
            }



        }
    }
}
