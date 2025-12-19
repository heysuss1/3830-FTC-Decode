package org.firstinspires.ftc.teamcode.PIDControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Pure FeedForward PID tuner")
public class VelocityPIDTester extends OpMode {
    Hardware robot = Hardware.getInstance();
    private Telemetry telemetryA;


    //Test: kP = 0.005;
    public static double kP, kI, kD;

    public static double kS = 1.26; //calculated by turning the flywheel
    public static double kV = 0.0024; //calculated by doing best line for power.

    double velError;
    public static double setpoint;
    double currentVel;
    double velFeedforward;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

    PIDController pid;
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
        pid = new PIDController(kP, kI, kD);

        robot.init(hardwareMap, telemetryA);
    }
    public double getBatteryVoltage(){
        return hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    //poopy butt
    public void loop() {
        currentVel = Math.abs(robot.shooter.getVelocity());
        velError = setpoint - currentVel;
        velFeedforward = feedforward.calculate(setpoint);
        double pidCorrection = pid.calculate(setpoint - currentVel);


        double output =velFeedforward + pidCorrection;
        output = (output/getBatteryVoltage());

        robot.shooter.setPower(Range.clip(output, -1, 1));

        telemetryA.addData("Current Velocity (rpm)", currentVel);
        telemetryA.addData("Velocity (raw tps)", robot.shooter.getTPSVelocity());
        telemetryA.addData("Setpoint", setpoint);
        telemetryA.addData("Voltage", getBatteryVoltage());


        telemetryA.update();

    }
}


