package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@Config
@TeleOp (name = "Shooter Params Tuner")
public class ShooterParamsTrainer extends LinearOpMode {

    Robot robot;
    public static double shooterTarget;
    public static double pitchTarget;

    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        robot.follower.setStartingPose(new Pose(123.5, 123.8, .74));
        waitForStart();
        while(opModeIsActive()){

            robot.shooter.setVelocityTarget(shooterTarget);
            robot.shooter.setPitchDegrees(pitchTarget);
            robot.shooter.pitchTask();
            robot.shooter.flywheelTask();

            robot.follower.update();
            telemetry.addData("Current RPM", robot.shooter.getVelocityRPM());
            telemetry.addData("Current Pitch", robot.shooter.getPitchDegrees());
            telemetry.addData("Distance To Goal", robot.getAimInfo().distance);
            telemetry.addData("Current position", robot.follower.getPose());
            telemetry.update();
        }
    }
}
