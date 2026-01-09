package org.firstinspires.ftc.teamcode.testCode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.TestHardware;
import org.firstinspires.ftc.teamcode.TestShooter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@TeleOp(name = "Voltage Encoder Test")
public class VoltageEncoderTester extends LinearOpMode {


    public static double shooterTarget;
    public static double pitchTarget;
    public static double kP, kI, kD, kF, iZone;
    public static double turretTarget;

    TestHardware robot;


    public void runOpMode() {
        robot = new TestHardware(hardwareMap, telemetry);
        robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);



        waitForStart();
        while (opModeIsActive()) {

            robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);
            robot.shooter.setTurretDegrees(turretTarget);
            robot.shooter.turretTask();
            telemetry.addData("Target Velocity (RPM)", turretTarget);
            telemetry.addData("Current Degrees ", robot.shooter.getTurretDegrees());
            telemetry.addData("Current Power", robot.shooter.getPrimaryTurretServo().getPower());
            telemetry.addData("Current Error", robot.shooter.getTurretController().getError());
            telemetry.addData("Raw position", robot.shooter.getTurretRawPose());
            telemetry.addData("Target", robot.shooter.getTurretTarget());
            telemetry.update();
        }
    }
}
