package org.firstinspires.ftc.teamcode.testCode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.TestHardware;
import org.firstinspires.ftc.teamcode.TestShooter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Voltage Encoder Test")
public class VoltageEncoderTester extends LinearOpMode {
    //    AnalogInput turretEncoder;
//    CRServo turretServo;
    int crossovers;

    TestHardware robot;
    private void crossoverCount(double prevValue, double currentValue, double  threshold) {
        if (prevValue - currentValue < threshold) { //1 to 0
            crossovers++;
        } else if (currentValue - prevValue > threshold) {  //0 to 1
            crossovers--;
        }
    }

    public void calculate(double prevValue, double currentValue, double threshold) {

        if (Math.abs(currentValue - prevValue) > threshold) {
            if (currentValue > prevValue) {
                crossovers--;
            }
            else if (currentValue < prevValue) {
                crossovers++;
            }
        }
    }

    public double getTurretRawPose() {
        return robot.shooter.getTurretEncoder().getVoltage()/robot.shooter.getTurretEncoder().getMaxVoltage();
    }

    public double getTurretDegree() {
        double rawPitchPos = (1-getTurretRawPose()) + 0.987 * crossovers;
        double encoderOffset = (rawPitchPos - TestShooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        double unconverted = (encoderOffset * TestShooter.Params.TURRET_DEGREES_PER_REV) + TestShooter.Params.PITCH_POSITION_OFFSET;
        return (unconverted + 180) % 360 - 180;
    }

    public void runOpMode(){
//        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
//        turretServo = hardwareMap.get(CRServo.class, "primaryTurretServo");
        robot = new TestHardware(hardwareMap, telemetry);

        waitForStart();
        double currentPos = 0, prevPos = 0;
        int loopCount = 0;

        while (opModeIsActive()) {

            prevPos = currentPos;
            currentPos = getTurretRawPose();
            calculate(currentPos, prevPos, 0.5);


            if(gamepad1.square){
                robot.shooter.getPrimaryTurretServo().setPower(0.3);
                Log.i("Robot", "Loop: " + loopCount + ", Prev: "+prevPos + ", Curr: "+currentPos + ", Difference: " + (prevPos - currentPos) + ", Crossovers: " + crossovers + ", Degrees: " + getTurretDegree());
            } else if (gamepad1.cross) {
                robot.shooter.getPrimaryTurretServo().setPower(-0.3);
                Log.i("Robot", "Loop: " + loopCount + ", " +
                        "Prev: "+prevPos + ", Curr: "+currentPos +
                        ", Difference: " + (prevPos - currentPos) +
                        ", Crossovers: " + crossovers +
                        ", Degrees: " + (getTurretDegree()));
            } else {
                robot.shooter.getPrimaryTurretServo().setPower(0);
            }
//            turretServo.setPower(1);
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Previous Pos", prevPos);
            telemetry.addData("Crossovers", crossovers);
            telemetry.addData("Current Power", robot.shooter.getPrimaryTurretServo().getPower());
            telemetry.addData("Alleged Degrees", getTurretDegree());
            telemetry.addData("Loop count", loopCount);
            telemetry.update();
            loopCount++;
        }
    }



}
