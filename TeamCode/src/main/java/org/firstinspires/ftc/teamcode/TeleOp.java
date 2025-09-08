package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems.IntakeNFTC;

public class TeleOp extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.driveTrain.setBrakeMode();
        boolean intakeOn = false;
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



        }
    }
}
