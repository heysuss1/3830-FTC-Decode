package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    public enum EditingModes {
        TEAM,
        AUTO_TYPE,
        START_DELAY,
    }

    public enum Team {
        RED,
        BLUE,
    }

    public enum AutoType {
        FAR_ZONE,
        CLOSE_ZONE,
    }

    Robot robot;

    AutoCommands autoCommand;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    private boolean editingConfig = true;
    private EditingModes editingMode = EditingModes.TEAM;

    private Team team = Team.RED;
    private AutoType autoType = AutoType.CLOSE_ZONE;
    private double startDelay = 0;

    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);

        while (editingConfig){
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if(gamepad1.circle && !previousGamepad1.circle)
            {
                editingConfig = false;
            }

            if(gamepad1.cross && !previousGamepad1.cross)
            {
                switch (editingMode)
                {
                    case TEAM: editingMode = EditingModes.AUTO_TYPE; break;
                    case AUTO_TYPE: editingMode = EditingModes.START_DELAY; break;
                    case START_DELAY: editingMode = EditingModes.TEAM; break;
                }
            }

            switch (editingMode) {
                case TEAM:
                    if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        team = Team.RED;
                    } else if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                        team = Team.BLUE;
                    }
                    break;
                case AUTO_TYPE:
                    if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        autoType = AutoType.CLOSE_ZONE;
                    } else if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                        autoType = AutoType.FAR_ZONE;
                    }
                    break;
                case START_DELAY:
                    if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        startDelay += 0.5;
                    } else if (gamepad1.left_bumper && !previousGamepad1.left_bumper) {
                        startDelay -= 0.5;
                    }
                    break;

            }
            telemetry.addData("Editing Mode", editingMode);
            telemetry.addData("Team", Robot.getTeam());
            telemetry.addData("Auto Type", autoType);
            telemetry.addData("Wait Time", startDelay);
            telemetry.update();
        }

        if (autoType == AutoType.CLOSE_ZONE) {
            autoCommand = new CmdTwelveCloseZoneAuto(robot, team, startDelay);
        } else {
            autoCommand = new CmdFarZoneAuto(robot, team, startDelay);
        }

        robot.shooter.setPitchDegrees(27.5);
        robot.shooter.setTurretDegrees(0.0);
        robot.intakeUptake.closeBlockingServo();
        robot.follower.setMaxPower(1);

        autoCommand.buildPaths();
        Robot.setTeam(team);
        Robot.setAutoType(autoType);
        waitForStart();
        autoCommand.startAutoCommand();
        while(opModeIsActive()){

            autoCommand.autonomousUpdate();
            robot.shooterTask.update();
            robot.shooter.shooterTask();
            robot.intakeUptake.intakeUptakeTask();
            robot.follower.update();

            Robot.setTeleOpStartPose(robot.follower.getPose());

            telemetry.addData("Current Auto State: ", autoCommand.getAutoState());
            telemetry.addData("Current Shooter State: ", robot.shooterTask.getShooterState());
            telemetry.addData("Is follower busy? ", robot.follower.isBusy());
            telemetry.addData("Shooter velocity: ", robot.shooter.getVelocityRPM());
            telemetry.addData("Is Flywheel on target: ", robot.shooter.isFlywheelOnTarget(Shooter.Params.SHOOTER_TOLERANCE_RPM) + ", Is pitch on target: " + robot.shooter.isPitchOnTarget(Shooter.Params.PITCH_TOLERANCE));
            telemetry.update();
        }


    }
    public static Pose convertAlliancePose(Pose pos, Team team) {
        double x = pos.getX();
        double y = pos.getY();
        double heading = pos.getHeading();

        if (team == Team.BLUE) {
            x = Robot.fieldParams.FIELD_LENGTH - x;
            heading = Math.floorMod((int)(180 - heading),360);
            //heading = ((180 - heading) % 360 + 360) % 360;
            //This is the same as the mod: heading = (180 - heading) >= 0 ? (180 - heading) : 360 + (180 - heading);
        }

        return new Pose(x, y, heading);
    }
}
