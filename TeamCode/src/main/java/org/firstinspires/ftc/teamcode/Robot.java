package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public static Pose teleOpStartPose;

    public static Pose RED_GOAL_SPOT = new Pose(143-20, 143-16.5, Math.toRadians(36));
    public static Pose BLUE_GOAL_SPOT = Robot.convertAlliancePose(RED_GOAL_SPOT);
    //public static Pose RED_FAR_SPOT = new Pose();
    //public static Pose BLUE_FAR_SPOT = Robot.convertAlliancePose(RED_FAR_SPOT);

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
    public static Pose convertAlliancePose(Pose pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        if (getTEAM() == Team.BLUE) {
            x = 2 * FIELD_CENTER_X - x;
            heading = Math.PI/2 - heading;
        }

        return new Pose(x, y, heading);
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
    public static Pose getTeleOpStartPose(){
        return teleOpStartPose;
    }
    public static void setTeleOpStartPose(Pose startPose){
        teleOpStartPose = startPose;
    }

    public AimInfo getAimInfo(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double x_goal = X_GOAL_RED;

        if (Robot.getTEAM() == Robot.Team.BLUE) {
            x_goal = X_GOAL_BLUE;
        } else {
            x_goal = X_GOAL_RED;
        }

        double deltaX = x_goal - robotX;
        double deltaY = Y_GOAL - robotY;
        double distanceToGoal = Math.hypot(deltaY, deltaX);
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return new AimInfo(distanceToGoal, angleToGoal);

    }

}
