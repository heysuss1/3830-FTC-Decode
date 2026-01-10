package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


public class Robot {

    public static class AimInfo {
        public double distance;
        public double angle;

        public AimInfo(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        public double getDistanceToGoal() {
            return distance;
        }

        public double getAngleToGoal() {
            return angle * -1;
        }
    }


    public enum Team{
        RED,
        BLUE,
    }
    public enum Motif{
        NULL,
        GPP,
        PGP,
        PPG
    }
    static Team TEAM = Team.RED;
    static Motif MOTIF = Motif.GPP;
    public final static boolean inComp = true;

    public static double cameraHeight = 6.7; //Inches
    public static double cameraAngle = 35; //degrees
    public final static double FIELD_CENTER_X = 72;

    public static double ballXOffset = 0, ballYOffset = 0;
    public final static double Y_GOAL = 143;
    public final static double X_GOAL_RED = 143;
    public final static double X_GOAL_BLUE = 0;
    public final DriveTrain driveTrain;
    public  Shooter shooter;
    public  Follower follower;
    public final Telemetry telemetry;
    public final IntakeUptake intakeUptake;

    public Robot(HardwareMap hwMap, Telemetry telemetry){
        follower = Constants.createFollower(hwMap);
        this.telemetry = telemetry;

        driveTrain = new DriveTrain(hwMap, telemetry);
        intakeUptake = new IntakeUptake(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry, this);
    }


    //the team is the alliance you're on for that match
    public static Pose convertAlliancePose(Pose pose){
        Pose newPose;
        double x = getTEAM() == Team.BLUE ? 2 * FIELD_CENTER_X - pose.getX(): pose.getX();
        double y = pose.getY();
        double heading = getTEAM() == Team.BLUE ? 180 - pose.getHeading() : pose.getHeading();
        newPose = new Pose(x, y, heading);
        return newPose;
    }
    public static Team getTEAM() {
        return TEAM;
    }

    public static void setMotif(Motif motif) {
        Robot.MOTIF = motif;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static void setTeam(Team team){
        TEAM = team;
    }

    public AimInfo getAimInfo(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double deltaX = Robot.getTEAM() == Robot.Team.BLUE ? robotX - X_GOAL_BLUE: robotX - X_GOAL_RED;
        double deltaY = robotY - Y_GOAL;
        double distanceToGoal = Math.hypot(deltaY, deltaX);
        double angleToGOal = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180;

        return new AimInfo(distanceToGoal, angleToGOal);

    }

}
