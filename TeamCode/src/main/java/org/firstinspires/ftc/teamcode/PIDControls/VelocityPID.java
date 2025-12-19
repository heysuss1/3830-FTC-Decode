package org.firstinspires.ftc.teamcode.PIDControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.Feedforward;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Pure FeedForward PID tuner")
public class VelocityPID extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;
    public static double kP, kI, kD;

    public static double kS = 0.1279; //calculated by turning the flywheel
    public static double kV = 0.000388; //calculated by doing best line for power.

    double velError;
    public static double setpoint;
    double currentVel;
    double velFeedforward;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

    PIDController pidf;
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pidf = new PIDController(kP, kI, kD);

        robot.init(hardwareMap, telemetryA);
    }
    public double getBatteryVoltage(){
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    //poopy butt
    public void loop() {
        currentVel = Math.abs(robot.shooter.getVelocity());
        velError = setpoint - currentVel;
        pidf.setPID(kP, kI, kD);
        velFeedforward = feedforward.calculate(setpoint) / getBatteryVoltage() ;
        double pidCorrection = pidf.calculate(currentVel, setpoint);

        double output =velFeedforward + pidCorrection;

        robot.shooter.setPower(output/getBatteryVoltage());

        telemetryA.addData("Current Velocity (rpm)", currentVel);
        telemetryA.addData("Setpoint", setpoint);
        telemetryA.addData("Voltage", getBatteryVoltage());


        telemetryA.update();

    }
}


