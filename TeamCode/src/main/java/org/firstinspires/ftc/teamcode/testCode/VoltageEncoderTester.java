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

    TestHardware

    public void runOpMode() {
        robot = new TestHardware(hardwareMap, telemetry);

        waitForStart();
        double currentPos = 0, prevPos = 0;
        int loopCount = 0;

        while (opModeIsActive()) {

            prevPos = currentPos;
            currentPos = getTurretRawPose();
            calculate(currentPos, prevPos, 0.5);


            if (gamepad1.square) {
                robot.shooter.getPrimaryTurretServo().setPower(0.3);
                Log.i("Robot", "Loop: " + loopCount + ", Prev: " + prevPos + ", Curr: " + currentPos + ", Difference: " + (prevPos - currentPos) + ", Crossovers: " + crossovers + ", Degrees: " + getTurretDegree());
            } else if (gamepad1.cross) {
                robot.shooter.getPrimaryTurretServo().setPower(-0.3);
                Log.i("Robot", "Loop: " + loopCount + ", " +
                        "Prev: " + prevPos + ", Curr: " + currentPos +
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
