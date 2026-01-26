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
    Double manualRpmOverride = null;

    public ShooterTask(Robot robot){
        this.robot = robot;
        timeoutTimer = new Timer();
    }

    public void startTask(Double manualRpm, double speedUpTimeout, double shootTimeout){
        taskFinished = false;
        this.speedUpTimeout = speedUpTimeout;
        this.shootTimeout = shootTimeout;
        manualRpmOverride = manualRpm;
        setShooterState(ShooterState.SPEEDING_UP);
    }

    public void startTask(Double manualRpm){
        startTask(manualRpm, DEFAULT_SPEEDUP_TIMEOUT, DEFAULT_SHOOT_TIMEOUT);
    }

    public void cancel(){
        taskFinished = true;
        robot.shooter.stopShooterMotor();
        robot.intakeUptake.closeBlockingServo();
        robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
    }

    public void setShooterState(ShooterState state){
        timeoutTimer.resetTimer();
        shooterState = state;
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
                }

                if (robot.shooter.isShooterReady() || (speedUpTimeout > 0.0 && timeoutTimer.getElapsedTimeSeconds() > speedUpTimeout))
                {
                    robot.intakeUptake.openBlockingServo();
                   setShooterState(ShooterState.SHOOTING);
                }
                break;

            case SHOOTING:
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (robot.intakeUptake.isUptakeEmpty() || (shootTimeout > 0.0 && timeoutTimer.getElapsedTimeSeconds() > shootTimeout)){
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
