package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;

public abstract class AutoCommands
{
    protected static final double intakePathSpeed = 0.7;
    protected static final double AUTO_RPM = 4000;

    protected enum AutoState{
        START,
        DRIVE_TO_SHOOTING_SPOT,
        SHOOTING,
        DRIVE_TO_GROUP,
        SLURPING_GROUP,
        DRIVE_TO_PARK,
        DRIVE_TO_GATE,
        SLURPING_FROM_GATE,
        STOP

    }

    protected final Robot robot;
    protected final Timer timer;

    protected final Auto.Team team;
    protected final Auto.AutoStrategy autoStrategy;
    protected final double waitTime;

    protected AutoState autoState;
    protected boolean isFirstTimePath = true;
    protected int shotCount = 0;

    public AutoCommands(Robot robot, Auto.Team team, Auto.AutoStrategy autoStrategy, double waitTime){
        this.robot = robot;
        this.team = team;
        this.autoStrategy = autoStrategy;
        this.waitTime = waitTime;
        timer = new Timer();
    }

    protected void startAutoCommand(){
        timer.resetTimer();
        autoState = AutoState.START;
    }

    protected void cancel(){
        robot.shooterTask.cancel();
        Robot.setTeleOpStartPose(robot.follower.getPose());
    }

    protected void setAutoState(AutoState state){
        autoState = state;
        timer.resetTimer();
    }

    protected AutoState getAutoState(){
        return autoState;
    }

    protected boolean isCycleStrategy() {
        return autoStrategy == Auto.AutoStrategy.CYCLE;
    }

    abstract void autonomousUpdate();

    abstract void buildPaths();
}
