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

    public ShooterTask(Robot robot){
        this.robot = robot;
        timeoutTimer = new Timer();
    }

    public void startTask(double speedUpTimeout, double shootTimeout){
        taskFinished = false;
        this.speedUpTimeout = speedUpTimeout;
        this.shootTimeout = shootTimeout;
        timeoutTimer.resetTimer();
        shooterState = ShooterState.SPEEDING_UP;
    }

    public void startTask(){
        startTask(DEFAULT_SPEEDUP_TIMEOUT, DEFAULT_SHOOT_TIMEOUT);
    }

    public void cancel(){
        taskFinished = true;
//        robot.intakeUptake.closeBlockingServo();
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
                robot.intakeUptake.openBlockingServo();
                if (robot.shooter.getAlwaysSetVelocity()){
                    robot.shooter.setVelocityTarget(robot.shooter.getCurrentVelocityTarget());
                } else {
                    robot.shooter.setVelocityTarget(robot.shooter.getVelocityTarget());
                }
                if (robot.shooter.isShooterReady() || (speedUpTimeout > 0.0 ||  timeoutTimer.getElapsedTimeSeconds() > speedUpTimeout))
                {
                    shooterState = ShooterState.SHOOTING;
                }
                break;

            case SHOOTING:

                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (robot.intakeUptake.isUptakeEmpty() || (shootTimeout > 0.0 && timeoutTimer.getElapsedTimeSeconds() > shootTimeout)){
//                    robot.intakeUptake.closeBlockingServo();
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
