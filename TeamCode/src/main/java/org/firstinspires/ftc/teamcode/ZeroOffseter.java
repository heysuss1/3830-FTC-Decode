package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Zero Offset Setter")
public class ZeroOffseter extends LinearOpMode {
    TestHardware robot;
    public void runOpMode(){
        robot = new TestHardware(hardwareMap, telemetry);
        waitForStart();
        double zeroOffset;
        while(opModeIsActive()){
            zeroOffset = robot.shooter.getTurretRawPose();
            robot.shooter.setZeroOffset(zeroOffset);
            Log.i("Velocity of turret", "Raw turret velocity: " + robot.shooter.getTurretRawPose());
            telemetry.addData("Zero Offset",  zeroOffset);
            telemetry.update();
        }
    }
}
