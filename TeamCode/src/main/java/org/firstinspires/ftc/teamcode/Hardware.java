package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


public class Hardware {

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

    public static Hardware INSTANCE = null;
    public final DriveTrain driveTrain;
    public final Shooter shooter;
    public final Follower follower;
    public final Telemetry telemetry;
    public final Transfer transfer;
    private HardwareMap hwMap;
    private double voltage;

    public Hardware(HardwareMap hwMap, Telemetry telemetry){

        for (LynxModule hub: hwMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hwMap);
        this.telemetry = telemetry;

        driveTrain = new DriveTrain(hwMap, telemetry);
        transfer = new Transfer(hwMap);
        shooter = new Shooter(hwMap, follower, telemetry);
    }


    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }



    public void setVoltage(){
        voltage = hwMap.voltageSensor.iterator().next().getVoltage();
    }

    public double getVoltage(){
        return voltage;
    }


    public AimInfo getAimInfo(){
        AimInfo aimInfo;
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double deltaX = RobotConstants.getTEAM() == RobotConstants.Team.BLUE ? robotX - RobotConstants.X_GOAL_BLUE: robotX - RobotConstants.X_GOAL_RED;
        double deltaY = robotY - RobotConstants.Y_GOAL;
        double distanceToGoal = Math.hypot(deltaY, deltaX);
        double angleToGOal = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180;

        return new AimInfo(distanceToGoal, angleToGOal);

    }




}
