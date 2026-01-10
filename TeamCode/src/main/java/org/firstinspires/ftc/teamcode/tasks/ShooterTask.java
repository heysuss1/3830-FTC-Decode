package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
public class ShooterTask {

    private Timer shooterTimer;
    private Robot robot;
    private int shotCounter;

    //TODO: refactor this class


    public ShooterTask(Robot robot){
        this.robot = robot;
        shooterTimer = new Timer();
        shotCounter = 0;
    }

    public enum ShooterState{
        START,
        SPEEDING_UP,
        SHOOTING,
        DONE
    }

    ShooterState shooterState = ShooterState.DONE;

    private void setShooterState(ShooterState state){
        shooterState = state;
        shooterTimer.resetTimer();
    }

    public void startShooterTask(){
        setShooterState(ShooterState.SPEEDING_UP);
    }

    public void cancelShooterUpdate(){
        setShooterState(ShooterState.DONE);
    }

    public ShooterState getShooterState(){
        return shooterState;
    }

    public boolean isFinished() {
        return shooterState == ShooterState.DONE;
    }


    public void update(double velTarget){
        switch (shooterState) {
            case START:
                //Intentionally falling through;

            case SPEEDING_UP:
                robot.shooter.setVelocityTarget(velTarget);
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM, Shooter.Params.PITCH_TOLERANCE)){
                    robot.intakeUptake.openBlockingServo();
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                /*TODO: check how much the velocity has dropped compared to the target velocity and change
                TODO: the pitch according to that, and remove has/hadball logic
                 */
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (shooterTimer.getElapsedTimeSeconds() > 2.5) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    setShooterState(ShooterState.DONE);
                }
                break;
            case DONE:
                robot.shooter.stopShooterMotor();
                robot.intakeUptake.closeBlockingServo();
                break;
        }
    }
}
