package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp (name = "Uptake Tester")
public class UptakeTester extends LinearOpMode {

    boolean uptakeIsOn = false;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        waitForStart();
       while (opModeIsActive()){
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.x && !previousGamepad1.x){
            uptakeIsOn = !uptakeIsOn;
            if (uptakeIsOn) {
                robot.shooter.startUptake();
            } else {
                robot.shooter.stopUptake();
            }
        }

    }
       }
}
