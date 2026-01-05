package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Turret PIDF tuner")
public class TuneTurretPidf extends OpMode {
    Robot robot;
    private Telemetry telemetryA;
    public static double kP, kI, kD, kF, iZone;
    public static Double targetPos;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot = new Robot(hardwareMap, telemetry);
    }

    public void loop() {
        robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot.shooter.setTurretDegrees(targetPos);
        telemetryA.addData("Target Position (degrees)", targetPos);
        telemetryA.addData("Current Position (degrees)", robot.shooter.getTurretDegrees());
        telemetryA.addData("Current Power", robot.shooter.getPrimaryTurretServo().getPower());
        telemetryA.addData("Current Error", robot.shooter.getTurretController().getError());
        telemetryA.update();
    }

}