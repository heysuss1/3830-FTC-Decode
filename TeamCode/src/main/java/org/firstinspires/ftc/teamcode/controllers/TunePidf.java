package org.firstinspires.ftc.teamcode.controllers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Velocity PID tuner")
public class TunePidf extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP, kI, kD, kF, iZone;
    public static int targetRpm;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.shooter.getShooterController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot.init(hardwareMap, telemetryA);
    }

    public void loop() {
        robot.shooter.getShooterController().setPidCoefficients(kP, kI, kD, kF, iZone);
        robot.shooter.setVelocityTarget(targetRpm);
        telemetryA.addData("Target Velocity (RPM)", targetRpm);
        telemetryA.addData("Current Velocity (RPM)", robot.shooter.getVelocity());
        telemetryA.addData("Current Power", 67);
        telemetryA.addData("Current Error", robot.shooter.getShooterController().getError());
    }

}