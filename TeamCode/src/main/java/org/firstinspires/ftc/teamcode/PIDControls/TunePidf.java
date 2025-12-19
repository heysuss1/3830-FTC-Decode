package org.firstinspires.ftc.teamcode.PIDControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Velocity PID tuner")
public class TunePidf extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP, kI, kD, kf, iZone;
    public static double targetRpm;
    double targetTps;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    PIDFController pidf;

    double velError;
    double currentVel;
    double velPower;

    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pidf = new PIDFController(kP, kI, kD, kf, iZone);
        robot.init(hardwareMap, telemetryA);
    }

    public void loop() {
        currentVel = Math.abs(robot.shooter.getVelocity());
        pidf.setPidCoefficients(kP, kI, kD, kf, iZone);
        velPower = pidf.calculate(targetRpm, currentVel);
        robot.shooter.setPower(velPower);
        telemetryA.addData("Current Velocity (rpm)", currentVel);
        telemetryA.addData("Calculated PID: ", velPower);
        telemetryA.addData("Target Velocity", targetRpm);
        telemetryA.update();
    }

}