package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class AutoCommands
{

    protected Robot robot;
    protected Timer pathTimer;
    protected Auto.Team team;
    protected final double waitTime;



    protected enum AutoState{
            START,
            DRIVE_TO_SHOOTING_SPOT,
            SHOOTING,
            DRIVE_TO_GROUP,
            SLURPING_GROUP,
            DRIVE_TO_PARK, // <-- dunno if we need this but just in case
            DRIVE_TO_GATE, // <--- these are for later when we get more balls than the pre-placed ones
            SLURPING_FROM_GATE,
            STOP

    }
    protected AutoState autoState;

    public AutoCommands(Robot robot, Auto.Team team, double waitTime){
        this.robot = robot;
        this.team = team;
        this.waitTime = waitTime;
        pathTimer = new Timer();
    }
    abstract void autonomousUpdate();

    abstract void buildPaths();

    protected void startAutoCommand(){
        pathTimer.resetTimer();
        autoState = AutoState.START;
    }

    protected void cancel(){
        robot.shooterTask.cancel();
        Robot.setTeleOpStartPose(robot.follower.getPose());
    }
    protected void setAutoState(AutoState state){
        autoState = state;
        pathTimer.resetTimer();
    }
    protected AutoState getAutoState(){
        return autoState;
    }
}
