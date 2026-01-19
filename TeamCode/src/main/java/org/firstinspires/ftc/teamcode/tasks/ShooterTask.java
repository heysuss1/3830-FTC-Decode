package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
public class ShooterTask {

    private Timer shotTimer;
    private Timer totalShooterTime;
    private Robot robot;
    private int shotCounter;

    //TODO: refactor this class


    public ShooterTask(Robot robot){
        this.robot = robot;
        shotTimer = new Timer();
        totalShooterTime = new Timer();
        shotCounter = 0;
    }

    public enum ShooterState{
        START,
        SPEEDING_UP,
        SHOOTING,
        WAITING,
        DONE
    }

    ShooterState shooterState = ShooterState.DONE;

    private void setShooterState(ShooterState state){
        shooterState = state;
        shotTimer.resetTimer();
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


    public void revUpShooterMotor(double velTarget){
        robot.shooter.setVelocityTarget(velTarget);
    }


    public void update(double velTarget){
        switch (shooterState) {
            case START:
                //Intentionally falling through;

            case SPEEDING_UP:
                robot.shooter.setVelocityTarget(velTarget);
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM)){
                    totalShooterTime.resetTimer();
                    robot.intakeUptake.openBlockingServo();
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                /*TODO: check how much the velocity has dropped compared to the target velocity and change
                TODO: the pitch according to that, and remove has/hadball logic
                 */

                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (shotTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intakeUptake.closeBlockingServo();
                    setShooterState(ShooterState.WAITING);
                }
                if (totalShooterTime.getElapsedTimeSeconds() > 2.7) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    setShooterState(ShooterState.DONE);
                }
                break;
            case WAITING:
                if (shotTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intakeUptake.openBlockingServo();
                }
                if (shotTimer.getElapsedTimeSeconds() > 0.6){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case DONE:
                robot.shooter.stopShooterMotor();
                robot.intakeUptake.closeBlockingServo();
                break;
        }
    }
    public void update(){
        switch (shooterState) {
            case START:
                //Intentionally falling through;

            case SPEEDING_UP:
                robot.shooter.setVelocityTarget(robot.shooter.getVelocityTarget());
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM)){
                    robot.intakeUptake.openBlockingServo();
                    totalShooterTime.resetTimer();
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                /*TODO: check how much the velocity has dropped compared to the target velocity and change
                TODO: the pitch according to that, and remove has/hadball logic
                 */
                robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.UPTAKING);

                if (shotTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intakeUptake.closeBlockingServo();
                    setShooterState(ShooterState.WAITING);
                }
                if (totalShooterTime.getElapsedTimeSeconds() > 3 || robot.intakeUptake.isUptakeEmpty()) {
                    robot.intakeUptake.setIntakeUptakeMode(IntakeUptake.intakeUptakeStates.OFF);
                    setShooterState(ShooterState.DONE);
                }
                break;
            case WAITING:
                if (shotTimer.getElapsedTimeSeconds() > 0.3){
                    robot.intakeUptake.openBlockingServo();
                }
                if (shotTimer.getElapsedTimeSeconds() > 0.6){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case DONE:
                robot.shooter.stopShooterMotor();
                robot.intakeUptake.closeBlockingServo();
                break;
        }
    }
}
