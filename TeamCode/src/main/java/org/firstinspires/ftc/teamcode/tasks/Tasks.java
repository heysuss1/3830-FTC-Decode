package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
public class Tasks {

    private Timer shooterTimer;
    private Robot robot;
    private int shotCounter;

    //TODO: refactor this class


    public Tasks(Robot robot){
        this.robot = robot;
        shooterTimer = new Timer();
        shotCounter = 0;
    }

    public enum ShooterState{
        OPEN_BLOCKING_SERVO,
        SPEEDING_UP,
        SHOOTING,
        WAITING,
        DONE
    }

    ShooterState shooterState = ShooterState.DONE;

    public void setShooterState(ShooterState state){
        shooterState = state;
        shooterTimer.resetTimer();
    }
    public void cancelShooterUpdate(){
        shooterState = ShooterState.DONE;
    }

    public ShooterState getShooterState(){
        return shooterState;
    }


    public void updateShooter(boolean hasBall, boolean hadBall){
        switch (shooterState) {
            case OPEN_BLOCKING_SERVO:
                robot.intakeUptake.openBlockingServo();
                robot.shooter.setVelocityTarget(3550.0);
                setShooterState(ShooterState.SPEEDING_UP);
                break;
            case SPEEDING_UP:
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM, Shooter.Params.PITCH_TOLERANCE, Shooter.Params.TURRET_TOLERANCE)){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                /*TODO: check how much the velocity has dropped compared to the target velocity and change
                TODO: the pitch according to that, and remove has/hadball logic
                 */
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);
                if (hadBall && !hasBall) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    shotCounter++;
                    setShooterState(ShooterState.WAITING);
                    return;
                }

                if (shotCounter >= 3) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    setShooterState(ShooterState.DONE);
                }
                if (shooterTimer.getElapsedTimeSeconds() > 1.5){
                    setShooterState(ShooterState.DONE);
                }
                break;
            case DONE:
                shotCounter = 0;
                robot.shooter.stopShooterSystem();
                robot.intakeUptake.closeBlockingServo();
                break;

        }
    }


//
    public void update(boolean hasBall, boolean hadBall){
        updateShooter(hasBall, hadBall);
        robot.intakeUptake.intakeUptakeTask();
    }


}
