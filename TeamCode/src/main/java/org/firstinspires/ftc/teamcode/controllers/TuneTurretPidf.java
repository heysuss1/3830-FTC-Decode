package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Turret PIDF tuner")
public class TuneTurretPidf extends OpMode {
    Hardware robot;
    private Telemetry telemetryA;
    public static double kP, kI, kD, kF, iZone;
    public static Double targetPos;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.shooter.getShooterController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot = new Hardware(hardwareMap, telemetry);
    }

    public void loop() {
        robot.shooter.getTurretController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot.shooter.setTurretDegrees(targetPos);
        telemetryA.addData("Target Position (degrees)", targetPos);
        telemetryA.addData("Current Velocity (RPM)", robot.shooter.getVelocityRPM());
        telemetryA.addData("Current Power", 67);
        telemetryA.addData("Current Error", robot.shooter.getShooterController().getError());
    }

}