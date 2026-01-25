package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autos.Auto;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;

public class Robot {

    public final static boolean inComp = true;

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

    static Auto.Team team;
    static Motif MOTIF = Motif.GPP;
    public static Pose teleOpStartPose;

    //TODO: Do you still need all this stuff between  48-54?
    public static Pose RED_GOAL_POSE = new Pose(123, 126.5, Math.toRadians(36));
    public static Pose BLUE_GOAL_POSE = (RED_GOAL_POSE); //TODO I HAVE TO CONVERT
    public static Pose RED_FAR_POSE = new Pose(60, 8, Math.toRadians(90));
    public static Pose BLUE_FAR_POSE = (RED_FAR_POSE); //I REMOVED CONVERSION
    public static String[] POSE_NAME_LIST = {"Red Goal", "Blue Goal", "Red Far Zone", "Blue Far Zone"};
    public static Pose[] POSE_LIST = {RED_GOAL_POSE, BLUE_GOAL_POSE, RED_FAR_POSE, BLUE_FAR_POSE};

    public static final class cameraParams {
        public static double cameraHeight = 6.7; //Inches
        public static double cameraAngle = 35; //degrees
    }


    public void resetPose(){
        if (team == Auto.Team.BLUE) follower.setPose(new Pose(135, 9, Math.PI/2));
        if (team == Auto.Team.RED   ) follower.setPose(new Pose(8, 8, Math.PI/2));
    }
    public static final class fieldParams {
        public final static double Y_GOAL = 141.5;
        public final static double X_GOAL_RED = 141.5;
        public final static double FIELD_LENGTH = 141.5;
        public final static double X_GOAL_BLUE = 0;
        public final static double FIELD_CENTER_X = 72;
    }

    public final Telemetry telemetry;
    public final DriveTrain driveTrain;
    public final Shooter shooter;
    public final IntakeUptake intakeUptake;
    public final Follower follower;
    public final ShooterTask shooterTask;

    public Robot(HardwareMap hwMap, Telemetry telemetry){
        follower = Constants.createFollower(hwMap);
        this.telemetry = telemetry;

        driveTrain = new DriveTrain(hwMap, telemetry);
        intakeUptake = new IntakeUptake(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry, this);

        shooterTask = new ShooterTask(this);
    }

    public static Auto.Team getTeam() {
        return team;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static Pose getTeleOpStartPose() {
        return teleOpStartPose;
    }

    public static void setMotif(Motif motif) {
        Robot.MOTIF = motif;
    }

    public static void setTeam(Auto.Team team) {
        Robot.team = team;
    }

    public static void setTeleOpStartPose(Pose startPose) {
        teleOpStartPose = startPose;
    }

    public AimInfo getAimInfo() {
        double robotX = follower.getPose().getX() + 3.5*Math.cos(follower.getHeading());
        double robotY = follower.getPose().getY() + 3.5*Math.sin(follower.getHeading());

        double x_goal;

        if (Robot.getTeam() == Auto.Team.BLUE) {
            x_goal = fieldParams.X_GOAL_BLUE;
        } else {
            x_goal = fieldParams.X_GOAL_RED;
        }

        double deltaX = x_goal - robotX;
        double deltaY = fieldParams.Y_GOAL - robotY;
        double distanceToGoal = Math.hypot(deltaY, deltaX);
        double angleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return new AimInfo(distanceToGoal, angleToGoal);

    }

    public boolean isInRevUpZone() {
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
