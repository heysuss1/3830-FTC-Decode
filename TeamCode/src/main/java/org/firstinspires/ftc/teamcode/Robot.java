package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;


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
            return angle;
        }
    }


    public enum Motif{
        NULL,
        GPP,
        PGP,
        PPG
    }
    static Auto.Team TEAM;
    static Motif MOTIF = Motif.GPP;
    public final static boolean inComp = true;

    public static Pose teleOpStartPose;

    public static Pose RED_GOAL_POSE = new Pose(123, 126.5, Math.toRadians(36));
    public static Pose BLUE_GOAL_POSE = (RED_GOAL_POSE); //TODO I HAVE TO CONVERT
    public static Pose RED_FAR_POSE = new Pose(60, 8, Math.toRadians(90));
    public static Pose BLUE_FAR_POSE = (RED_FAR_POSE); //I REMOVED CONVERSION

    public static String[] POSE_NAME_LIST = {"Red Goal", "Blue Goal", "Red Far Zone", "Blue Far Zone"};
    public static Pose[] POSE_LIST = {RED_GOAL_POSE, BLUE_GOAL_POSE, RED_FAR_POSE, BLUE_FAR_POSE};

    public static double cameraHeight = 6.7; //Inches
    public static double cameraAngle = 35; //degrees
    public final static double FIELD_CENTER_X = 72;

    public static double ballXOffset = 0, ballYOffset = 0;
    public final static double Y_GOAL = 141.5;
    public final static double X_GOAL_RED = 141.5;
    public final static double FIELD_LENGTH = 141.5;

    public final static double X_GOAL_BLUE = 0;
    public final DriveTrain driveTrain;
    public  Shooter shooter;
    public ShooterTask shooterTask;
    public  Follower follower;
    public final Telemetry telemetry;
    public final IntakeUptake intakeUptake;

    public Robot(HardwareMap hwMap, Telemetry telemetry){
        follower = Constants.createFollower(hwMap);
        this.telemetry = telemetry;

        driveTrain = new DriveTrain(hwMap, telemetry);
        intakeUptake = new IntakeUptake(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry, this);

        shooterTask = new ShooterTask(this);
    }


    //the team is the alliance you're on for that match


    public static Auto.Team getTEAM() {
        return TEAM;
    }

    public static void setMotif(Motif motif) {
        Robot.MOTIF = motif;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static void setTeam(Auto.Team team){
        TEAM = team;
    }
    public static Pose getTeleOpStartPose(){
        return teleOpStartPose;
    }
    public static void setTeleOpStartPose(Pose startPose){
        teleOpStartPose = startPose;
    }

    public AimInfo getAimInfo(){
        double robotX = follower.getPose().getX() + 3.5*Math.cos(follower.getHeading());
        double robotY = follower.getPose().getY() + 3.5*Math.sin(follower.getHeading());

        double x_goal;

        if (Robot.getTEAM() == Auto.Team.BLUE) {
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
    public boolean isInRevUpZone(){
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double multiplier = 141.5/144;
        boolean isAboveRightLine = y > x;
        boolean isAboveLeftLine = y > -x + 144 * multiplier;

        boolean isInClozeZone = isAboveLeftLine && isAboveRightLine;

        isAboveLeftLine = y < x - 48 * multiplier;
        isAboveRightLine = y < x *-1 + 102 * multiplier;

        boolean isInFarZone = (isAboveLeftLine && isAboveRightLine);
        return (isInClozeZone || isInFarZone);
    }

}
