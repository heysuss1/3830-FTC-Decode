package org.firstinspires.ftc.teamcode.PIDControls;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
@TeleOp(name = "Velocity PID tuner")
public class VelocityPID extends OpMode {
    Hardware robot = Hardware.getInstance();
        private Telemetry telemetryA;
        public static double kP, kI, kD, kF;

        double velError;

        public static int targetVel;
//    public static int armVerticalPos;

        //    private double gravityComp;
//    double output;
        public static int setpoint;
        double currentVel;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        com.rowanmcalpin.nextftc.core.control.controllers.PIDFController pidf;

        public void init(){
            telemetryA = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());
            pidf = new PIDFController(kP, kI, kD);
            robot.init(hardwareMap, telemetryA);
        }
        public void loop(){
            currentVel = robot.shooter.getCannonVelocity();
            velError = setpoint - currentVel;
            pidf.calculate(0, velError);
            telemetryA.addData("Current Error: ", velError);
            telemetryA.addData("Current Velocity", currentVel);
            telemetryA.update();

        }
//    public void extend(int target)
//        gravityComp = -0.001 * Math.sin((Math.PI*robot.armVertical.getCurrentPosition())/7600);
//        output = pidf.calculate(robot.armExtension.getCurrentPosition(), target);
//        robot.armExtension.setPower(output);
//    }
    }


