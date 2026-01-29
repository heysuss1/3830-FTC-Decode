package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.autos.Auto;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {


    public static double FIELD_X = Robot.fieldParams.Y_GOAL;
    public static double FIELD_Y = Robot.fieldParams.X_GOAL_RED;
    public static double turret_offset = Shooter.Params.TURRET_POSITION_OFFSET;
    public static double blue_line = Robot.fieldParams.BLUE_REV_LINE_Y_INT;



    final double MAX_SPEED = 1.0; //See if you driver can handle 1.0, it should net faster cycles.
    final double SLOW_SPEED = 0.3;
    final double CLOSE_ZONE_RPM = 3700.0;
    final double FAR_ZONE_RPM = 4500.0;

    /*
    right trigger - slow mode
    right bumper - Intake toggle
    circle - outtake
    x - shot
    cross - cancel shooting
    back - toggle always set velocity
    dpad right - toggle always aim pitch
    left bumper - toggle always aim turret
    dpad left - reset pose
    dpad up - increase manual RPM by 100
    dpad down - decrease manual RPM by 100
    */

    Robot robot;
    Timer loopTimer;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    double currentTime = 0, lastTime = 0;
    boolean intakeOn = false;
    Double manualRPM = null;

    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        loopTimer = new Timer();

        robot.driveTrain.setSpeed(MAX_SPEED);
        Pose startingPose = (new Pose(114, 123, Math.toRadians(42)));

        robot.driveTrain.setBrakeMode();

        while (opModeInInit()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                Robot.setTeam(Auto.Team.RED == Robot.getTeam() ? Auto.Team.BLUE : Auto.Team.RED);
            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                Robot.setAutoType(Auto.AutoType.CLOSE_ZONE == Robot.getAutoType() ? Auto.AutoType.FAR_ZONE : Auto.AutoType.CLOSE_ZONE);
            }

            telemetry.addData("Team: ", Robot.getTeam());
            telemetry.addData("Auto Type: ", Robot.getAutoType());
            telemetry.update();
        }

//        if (Robot.getTeleOpStartPose() == null) {
//            robot.resetPose();
//        } else {
//            robot.follower.setStartingPose(Robot.getTeleOpStartPose());
//        }

        Shooter.alwaysSetVelocity = true;

        Shooter.alwaysAimTurret = false;
        robot.follower.setStartingPose(startingPose);
        telemetry.addData("Robot Pose", robot.follower.getPose());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            //Tracks loop times
            lastTime = currentTime;
            currentTime = loopTimer.getElapsedTime();

            Robot.fieldParams.Y_GOAL = FIELD_Y;
            Robot.fieldParams.X_GOAL_RED = FIELD_X;
            Shooter.Params.TURRET_POSITION_OFFSET = turret_offset;
            Robot.fieldParams.BLUE_REV_LINE_Y_INT = blue_line;


            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.right_trigger > 0.1)
                robot.driveTrain.setSpeed(SLOW_SPEED);
            else {
                robot.driveTrain.setSpeed(MAX_SPEED);
            }

            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                intakeOn = !intakeOn;
                robot.intakeUptake.setIntakeUptakeMode(
                        intakeOn ? IntakeUptake.intakeUptakeStates.INTAKING
                                : IntakeUptake.intakeUptakeStates.OFF
                );
            }

            if (currentGamepad1.circle && !previousGamepad1.circle) {
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OUTTAKING);
                intakeOn = false;
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                robot.shooterTask.startTask(manualRPM);
                intakeOn = true;
            }

            if (currentGamepad1.cross && !previousGamepad1.cross) {
                robot.shooterTask.cancel();
                intakeOn = false;
            }

            if (currentGamepad1.back && !previousGamepad1.back){
                Shooter.alwaysSetVelocity = !Shooter.alwaysSetVelocity;  // Toggle the boolean
                if (!Shooter.alwaysSetVelocity){
                    manualRPM = CLOSE_ZONE_RPM;
                }
                else manualRPM = null;
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                Shooter.alwaysAimPitch = !Shooter.alwaysAimPitch;  // Toggle the boolean
                if (!Shooter.alwaysAimPitch)
                {
                    robot.shooter.setPitchDegrees(27.0);
                }
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                Shooter.alwaysAimTurret = !Shooter.alwaysAimTurret;  // Toggle the boolean
                if (!Shooter.alwaysAimTurret){
                    robot.shooter.setTurretDegrees(0.0);
                }
            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                robot.resetPose();
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up && !Shooter.alwaysSetVelocity){
                manualRPM += 100.0;
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down && !Shooter.alwaysSetVelocity){
                manualRPM -= 100.0;
            }

            robot.shooterTask.update();
            robot.follower.update();
            robot.shooter.shooterTask();
            robot.intakeUptake.intakeUptakeTask();
            robot.driveTrain.driveTask(currentGamepad1);

            telemetry.addData("Shooter vel: ", robot.shooter.getVelocityRPM());
            telemetry.addData("Loop Time", currentTime - lastTime);
            telemetry.addData("x: ", robot.follower.getPose().getX());
            telemetry.addData("y: ", robot.follower.getPose().getY());
            telemetry.addData("Heading: ", robot.follower.getHeading());
            telemetry.addData("Shooter param rpm", robot.shooter.getVelocityTarget());
            telemetry.addData("Current Turret Degrees", robot.shooter.getTurretDegrees());
            telemetry.addData("Number of big BLACK balls", robot.intakeUptake.getNumberOfBallsStored());
            telemetry.addData("Shooter Velocity Target", robot.shooter.getVelocityTarget());
            telemetry.addData("Is in rev up zone", robot.isInRevUpZone());
            telemetry.addData("Shooter State", robot.shooterTask.getShooterState());
            telemetry.addData("Manual RPM", manualRPM);
            telemetry.addData("Always set velocity?", Shooter.alwaysSetVelocity);
            telemetry.addData("Always set turret?", Shooter.alwaysAimTurret);
            telemetry.addData("Is Flywheel on target: ", robot.shooter.isFlywheelOnTarget(Shooter.Params.SHOOTER_TOLERANCE_RPM) + ", Is pitch on target: " + robot.shooter.isPitchOnTarget(Shooter.Params.PITCH_TOLERANCE) + ", Is turret on target: ", robot.shooter.isTurretOnTarget(Shooter.Params.TURRET_TOLERANCE));
            telemetry.addData("Shooter empty", robot.intakeUptake.isUptakeEmpty());
            telemetry.update();
        }
    }
}
