package org.firstinspires.ftc.teamcode.PIDControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Velocity PID tuner")
public class VelocityPID extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP, kI, kD;

    double velError;
    public static int setpoint;
    double currentVel;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    com.rowanmcalpin.nextftc.core.control.controllers.PIDFController pidf;

    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        robot.shooter.shootingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pidf = new PIDFController(kP, kI, kD);
        robot.init(hardwareMap, telemetryA);
    }

    public void loop() {
        currentVel = robot.shooter.getVelocity();
        velError = setpoint - currentVel;
        pidf.setKP(kP);
        pidf.setKI(kI);
        pidf.setKD(kD);
        double velPower = pidf.calculate(currentVel, setpoint);
        robot.shooter.setPower(velPower);
        telemetryA.addData("Current Error: ", velError);
        telemetryA.addData("Current Velocity", currentVel);
        telemetryA.update();

    }
}


