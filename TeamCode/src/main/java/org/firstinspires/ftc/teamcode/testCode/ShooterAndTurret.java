package org.firstinspires.ftc.teamcode.testCode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TestHardware;
import org.firstinspires.ftc.teamcode.TestShooter;

@Config
@TeleOp(name = "Test to turn turret around and work")
public class ShooterAndTurret extends LinearOpMode {
    TestHardware robot;
    public static double pitchTarget;
    public static double gearRatio;
    public void runOpMode(){
        robot = new TestHardware(hardwareMap, telemetry);
        robot.follower.setStartingPose(new Pose(96, 96, 0));
//        robot.shooter.setPitchDegrees(TestShooter.Params.MAX_PITCH_DEGREES);
        waitForStart();
//        robot.shooter.setPitchDegrees(TestShooter.Params.MIN_PITCH_DEGREES);
        while (opModeIsActive()){

            robot.shooter.setPitchDegrees(pitchTarget, gearRatio);
            robot.shooter.pitchTask();
            Log.i("Pitch Voltage", "Max Voltage: " + robot.shooter.getPitchEncoder().getMaxVoltage() + ", Curr pos w/ max voltage: " + (robot.shooter.getPitchEncoder().getVoltage()/
                    robot.shooter.getPitchEncoder().getMaxVoltage()) +
                    ", curr pos with 3.3: " + (robot.shooter.getPitchEncoder().getVoltage()/3.3));

//        robot.shooter.flywheelTask();
         robot.follower.update();
        }
    }
}
