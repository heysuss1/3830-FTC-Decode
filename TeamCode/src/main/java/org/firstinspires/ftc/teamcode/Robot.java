package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


public class Robot {

    public class AimInfo {
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
    static Team TEAM;
    static Motif MOTIF;
    public final static boolean inComp = false;

    public static Team getTEAM() {
        return TEAM;
    }
    public static double cameraHeight = 6.7; //Inches
    public static double cameraAngle = 35; //degrees
    public static void setMotif(Motif motif) {
        Robot.MOTIF = motif;
    }

    public static Motif getMotif() {
        return MOTIF;
    }

    public static void setTeam(Team team){
        TEAM = team;
    }

    public static double ballXOffset = 0, ballYOffset = 0;
    public final static double Y_GOAL = 142;
    public final static double X_GOAL_RED = 142;
    public final static double X_GOAL_BLUE = 2;
    public final DriveTrain driveTrain;
    public final Shooter shooter;
    public final Follower follower;
    public final Telemetry telemetry;
    public final IntakeUptake intakeUptake;
    public final VoltageSensor voltageSensor;

    public Robot(HardwareMap hwMap, Telemetry telemetry){

        for (LynxModule hub: hwMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hwMap);
        this.telemetry = telemetry;

        voltageSensor = hwMap.voltageSensor.iterator().next();
        driveTrain = new DriveTrain(hwMap, telemetry);
        transfer = new IntakeUptake(hwMap, telemetry);
        shooter = new Shooter(hwMap, telemetry, this);
    }


    public double getVoltage(){
        return voltageSensor.getVoltage();
    }

    public Follower getFollower() {
        return follower;
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
