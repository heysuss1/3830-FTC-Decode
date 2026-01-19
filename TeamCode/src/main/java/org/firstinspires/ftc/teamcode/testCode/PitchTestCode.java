package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;



@Config
@TeleOp(name = "Pitch Position Test")
public class PitchTestCode extends LinearOpMode {
    AnalogInput pitchEncoder;
    Servo pitchServo;

    double servoPosition = 0;
    public static double PITCH_GEAR_RATIO = .177; //.208 //(15.0/173) * (48.0/20)


    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / 3.3);
        return rawPitchToDegrees(rawPitchPos);
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * (PITCH_GEAR_RATIO * 360)) + Shooter.Params.PITCH_POSITION_OFFSET;
    }

    public void runOpMode(){
        pitchEncoder = hardwareMap.get(AnalogInput.class, "pitchEncoder");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
        waitForStart();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        while (opModeIsActive()){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                servoPosition += 0.1;
            } else if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                servoPosition -= 0.1;
            } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                servoPosition -= 0.01;
            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                servoPosition += 0.01;
            }
            pitchServo.setPosition(servoPosition);
            telemetry.addData("Servo position", pitchEncoder.getVoltage()/3.3);
            telemetry.addData("pitch degree", getPitchDegrees());
            telemetry.update();

        }
    }
}
