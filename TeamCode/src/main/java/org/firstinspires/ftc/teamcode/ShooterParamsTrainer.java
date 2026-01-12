package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;


@Config
@TeleOp (name = "Shooter Params Tuner")
public class ShooterParamsTrainer extends LinearOpMode {

    Robot robot;
    public static double shooterTarget;
    public static double pitchTarget;
    ShooterTask task;

    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);
        task = new ShooterTask(robot);
        robot.follower.setStartingPose(new Pose(143-20, 143-16.5, .74));
        waitForStart();
        while(opModeIsActive()){

            robot.shooter.setPitchDegrees(pitchTarget);
            robot.shooter.pitchTask();
            robot.shooter.flywheelTask();
            task.update(shooterTarget);
            robot.follower.update();

            if (gamepad1.square) {
                task.startShooterTask();
            }

            if (gamepad1.circle)
            {
                robot.intakeUptake.closeBlockingServo();
            }

            if (gamepad1.cross)
            {
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.INTAKING);
            }

            if (gamepad1.right_bumper)
            {
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
            }

            if (gamepad1.left_bumper) {
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OUTTAKING);
            }

            robot.shooter.turretTask();
            if (gamepad1.circle){
                robot.shooter.setTurretDegrees((Double)null);
            } else {
                robot.shooter.setTurretDegrees(robot.getAimInfo().angle);
            }
            robot.intakeUptake.intakeUptakeTask();
            telemetry.addData("Current RPM", robot.shooter.getVelocityRPM());
            telemetry.addData("Current Pitch", robot.shooter.getPitchDegrees());
            telemetry.addData("Distance To Goal", robot.getAimInfo().distance);
            telemetry.addData("Angle To Goal", (robot.getAimInfo().angle - Math.toDegrees(robot.follower.getHeading())) * -1);
            telemetry.addData("Current position", robot.follower.getPose());
            telemetry.addData("Target", robot.shooter.getPitchTarget());
            telemetry.addData("Current turret degrees", robot.shooter.getTurretDegrees());
            telemetry.addData("Current servo pos raw", robot.shooter.getRawPitchPos());
            telemetry.addData("Task state", task.getShooterState());
            telemetry.addData("Is Flywheel on target: ", robot.shooter.isFlywheelOnTarget(Shooter.Params.SHOOTER_TOLERANCE_RPM) + ", Is pitch on target: " + robot.shooter.isPitchOnTarget(Shooter.Params.PITCH_TOLERANCE));
            telemetry.update();
        }
    }
}
