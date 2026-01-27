package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;



@Config
@TeleOp(name = "Pitch Position Test")
public class PitchTestCode extends LinearOpMode {


    Robot robot;
    public static double servoPosition = 0;
    public static double PITCH_GEAR_RATIO = .177; //.208 //(15.0/173) * (48.0/20)


    public double getPitchDegrees() {
        return rawPitchToDegrees(robot.shooter.getRawPitchPos());
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * (PITCH_GEAR_RATIO * 360)) + Shooter.Params.PITCH_POSITION_OFFSET;
    }

    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
//            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
//                servoPosition += 0.1;
//            } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
//                servoPosition -= 0.1;
//            } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
//                servoPosition -= 0.01;
//            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
//                servoPosition += 0.01;
//            }
            robot.shooter.setPitchDegrees(servoPosition);
            robot.shooter.pitchTask();
            telemetry.addData("Servo position", robot.shooter.getRawPitchPos());
            telemetry.addData("pitch degree", getPitchDegrees());
            telemetry.update();

        }
    }
}
