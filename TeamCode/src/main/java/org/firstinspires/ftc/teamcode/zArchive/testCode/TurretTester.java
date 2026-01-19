package org.firstinspires.ftc.teamcode.zArchive.testCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name="Turret Tester")
@Disabled
public class TurretTester extends LinearOpMode {

    Robot robot;

    int crossovers;
    private void crossoverCount(double prevValue, double currentValue) {
        if (prevValue - currentValue > 0.8) { //1 to 0
            crossovers++;
        } else if (currentValue - prevValue > 0.8) {  //0 to 1
            crossovers--;
        }
    }

    public double getTurretDegree() {
        double rawPitchPos = robot.shooter.getTurretRawPose();
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        double currentDegrees = (encoderOffset * Shooter.Params.TURRET_DEGREES_PER_REV) + Shooter.Params.PITCH_POSITION_OFFSET;
        return currentDegrees + crossovers * Shooter.Params.TURRET_DEGREES_PER_REV;
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        waitForStart();
        double currentPos = 0, prevPos = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentPos = robot.shooter.getTurretRawPose();
            crossoverCount(currentPos, prevPos);
            prevPos = currentPos;

            robot.telemetry.addData("Current Pos", currentPos);
            robot.telemetry.addData("Previous Pos", prevPos);
            robot.telemetry.addData("Crossovers", crossovers);
            robot.telemetry.addData("Alleged Degrees", getTurretDegree());
        }
    }
}
