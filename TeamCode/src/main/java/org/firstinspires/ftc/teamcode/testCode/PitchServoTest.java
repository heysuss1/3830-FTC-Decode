package org.firstinspires.ftc.teamcode.testCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@TeleOp (name = "Basic Pitch Test")
public class PitchServoTest extends LinearOpMode {
    Robot robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry telemetryA;

    public static double PITCH_GEAR_RATIO = .177; //.208 //(15.0/173) * (48.0/20)

    public double getPitchDegrees() {
        return rawPitchToDegrees(robot.shooter.getRawPitchPos());
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * (PITCH_GEAR_RATIO * 360)) + Shooter.Params.PITCH_POSITION_OFFSET;
    }
    public static double pitchServoPosition = 0;
    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        telemetryA = new MultipleTelemetry(dashboard.getTelemetry(), telemetry);
        waitForStart();
        while (opModeIsActive()){
            robot.shooter.getPitchServo().setPosition(pitchServoPosition);
            telemetryA.addData("Servo position", robot.shooter.getRawPitchPos());
            telemetryA.addData("pitch degree", getPitchDegrees());
            telemetryA.update();
        }
    }
}
