package org.firstinspires.ftc.teamcode;

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
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        robot.driveTrain.setBrakeMode();
        robot.driveTrain.setSpeed(1);
        boolean intakeOn = false;
        boolean rightBumper = false;
        waitForStart();
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
                //shoot
            }



        }
    }
}
