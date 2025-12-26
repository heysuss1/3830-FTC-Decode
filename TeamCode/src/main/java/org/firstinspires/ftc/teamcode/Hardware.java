package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Vision.Vision;

public class Hardware {

    public static Hardware INSTANCE = null;
    public DriveTrain driveTrain = new DriveTrain();
    public Shooter shooter;

    Follower follower;

//    public Turret turret = new Turret();
    public Transfer transfer;
    private HardwareMap hwMap;
    private double voltage;
    public Vision vision = new Vision();
    public double maxSpeed;






    public static Hardware getInstance(){
        if (INSTANCE == null){
            INSTANCE = new Hardware();
        }
        return INSTANCE;
    }



    public void setVoltage(){
        voltage = hwMap.voltageSensor.iterator().next().getVoltage();
    }

    public double getDistance( ){
        double x_goal =  RobotConstants.getTEAM() == RobotConstants.Team.BLUE ? RobotConstants.X_GOAL_BLUE : RobotConstants.X_GOAL_RED;
        return Math.sqrt(Math.pow(follower.getPose().getY()-RobotConstants.Y_GOAL, 2)+Math.pow(follower.getPose().getY()-x_goal,2));
    }

    public double getVoltage(){
        return voltage;
    }
    public void init(HardwareMap hwMap, Telemetry telemetry){

        follower = Constants.createFollower(hwMap);
        this.hwMap = hwMap;
        driveTrain.init(hwMap, telemetry);
        transfer = new Transfer(hwMap);
        shooter = new Shooter(hwMap, follower);
//        turret.init(hwMap);
    }
}
