package org.firstinspires.ftc.teamcode.testCode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "Voltage Encoder Test")
public class VoltageEncoderTester extends LinearOpMode {
    AnalogInput turretEncoder;
    CRServo turretServo;
    int crossovers;

    private void crossoverCount(double prevValue, double currentValue, double threshold) {
        if (prevValue - currentValue > threshold) { //1 to 0
            crossovers++;
        } else if (currentValue - prevValue > threshold) {  //0 to 1
            crossovers--;
        }
    }

    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/3.3;
    }

    public double getTurretDegree() {
        double rawPitchPos = getTurretRawPose();
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        double currentDegrees = (encoderOffset * Shooter.Params.TURRET_DEGREES_PER_REV) + Shooter.Params.PITCH_POSITION_OFFSET;
        return currentDegrees + crossovers * Shooter.Params.TURRET_DEGREES_PER_REV;
    }

    public void runOpMode(){
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        turretServo = hardwareMap.get(CRServo.class, "turretServo");

        waitForStart();
        double currentPos = 0, prevPos = 0;
        int loopCount = 0;

        while (opModeIsActive()) {

            prevPos = currentPos;
            currentPos = getTurretRawPose();
            crossoverCount(currentPos, prevPos, 0.5);


            if(gamepad1.square){
                turretServo.setPower(0.3);
            } else if (gamepad1.cross) {
                turretServo.setPower(-0.3);
            }
            turretServo.setPower(1);
            Log.i("Robot", "Loop: " + loopCount + ", Prev: "+prevPos + ", Curr: "+currentPos + ", Difference: " + (prevPos - currentPos) + ", Crossovers: " + crossovers);
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Previous Pos", prevPos);
            telemetry.addData("Crossovers", crossovers);
            telemetry.addData("Alleged Degrees", getTurretDegree());
            telemetry.addData("Loop count", loopCount);
            telemetry.update();
            loopCount++;
        }
    }
}
