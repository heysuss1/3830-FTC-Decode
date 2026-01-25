package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
public class ShooterTask {

    public static final double DEFAULT_SPEEDUP_TIMEOUT = 2.0;
    public static final double DEFAULT_SHOOT_TIMEOUT = 1.0;

    public enum ShooterState{
        SPEEDING_UP,
        SHOOTING,
        DONE,
        IDLE
    }

    private final Robot robot;
    private final Timer timeoutTimer;

    ShooterState shooterState = ShooterState.IDLE;
    boolean taskFinished;
    double speedUpTimeout;
    double shootTimeout;

    public Double manualRpmOverride;

    public ShooterTask(Robot robot){
        this.robot = robot;
        timeoutTimer = new Timer();
    }

    public void startTask(double speedUpTimeout, double shootTimeout, Double manualRpm){
        taskFinished = false;
        this.speedUpTimeout = speedUpTimeout;
        this.shootTimeout = shootTimeout;
        timeoutTimer.resetTimer();
        shooterState = ShooterState.SPEEDING_UP;
        manualRpmOverride = manualRpm;
    }

    public void startTask(Double manualRpm){
        startTask(DEFAULT_SPEEDUP_TIMEOUT, DEFAULT_SHOOT_TIMEOUT, manualRpm);
    }

    public void cancel(){
        taskFinished = true;
        robot.intakeUptake.closeBlockingServo();
    }


    public void setShooterState(ShooterState state){
        shooterState = state;
        timeoutTimer.resetTimer();
    }
    public ShooterState getShooterState(){
        return shooterState;
    }

    public boolean isFinished() {
        return taskFinished;
    }

    public void update(){
        switch (shooterState) {
            case SPEEDING_UP:
                if (manualRpmOverride != null) {
                    robot.shooter.setVelocityTarget(manualRpmOverride);
                } else{
                    robot.shooter.setVelocityTarget(robot.shooter.getCurrentVelocityTarget());
                }

                if (robot.shooter.isShooterReady() || (speedUpTimeout > 0.0 && timeoutTimer.getElapsedTimeSeconds() > speedUpTimeout))
                {
                   setShooterState(ShooterState.SHOOTING);
                }
                break;

            case SHOOTING:
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (robot.intakeUptake.isUptakeEmpty() || (shootTimeout > 0.0 && timeoutTimer.getElapsedTimeSeconds() > shootTimeout)){
                    robot.intakeUptake.closeBlockingServo();
                    shooterState = ShooterState.DONE;
                }
                break;
            case DONE:
                cancel();
                break;
            case IDLE:
                break;
        }
    }
}
