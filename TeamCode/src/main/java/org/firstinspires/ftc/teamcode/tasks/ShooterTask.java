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

    public void setShooterState(ShooterState state){
        shooterState = state;
        shooterTimer.resetTimer();
    }

    public void startShooterTask(){
        setShooterState(ShooterState.START);
    }
    public void cancelShooterUpdate(){
        setShooterState(ShooterState.DONE);
    }

    public ShooterState getShooterState(){
        return shooterState;
    }


    public void update(){
        switch (shooterState) {
            case START:
                //Intentionally falling through;

            case SPEEDING_UP:
                robot.shooter.setVelocityTarget(robot.shooter.getCurrentShooterVelTarget());
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM, Shooter.Params.PITCH_TOLERANCE, Shooter.Params.TURRET_TOLERANCE)){
                    robot.intakeUptake.openBlockingServo();
                    setShooterState(ShooterState.SHOOTING);

                }
                break;
            case SHOOTING:
                /*TODO: check how much the velocity has dropped compared to the target velocity and change
                TODO: the pitch according to that, and remove has/hadball logic
                 */
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (robot.intakeUptake.isUptakeEmpty()) {
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
