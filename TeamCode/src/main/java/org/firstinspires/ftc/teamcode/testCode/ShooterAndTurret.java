package org.firstinspires.ftc.teamcode.testCode;

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
    public void runOpMode(){
        ElapsedTime time = new ElapsedTime();
        robot = new TestHardware(hardwareMap, telemetry);
        robot.follower.setStartingPose(new Pose(96, 96, 0));
        robot.shooter.setPitchDegrees(TestShooter.Params.MAX_PITCH_DEGREES);
        waitForStart();
        robot.shooter.setPitchDegrees(TestShooter.Params.MIN_PITCH_DEGREES);
        while (opModeIsActive()){

            robot.shooter.setPitchDegrees(pitchTarget);
            robot.shooter.pitchTask();
//        robot.shooter.flywheelTask();
         robot.follower.update();
        }
    }
}
