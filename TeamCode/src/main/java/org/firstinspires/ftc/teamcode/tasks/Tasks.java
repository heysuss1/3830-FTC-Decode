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




//    public void updateTransfer(boolean hasBall){
//        switch (transferState){
//            case OFF:
//                robot.intakeUptake.stopTransfer();
//                break;
//            case INTAKE:
//                robot.intakeUptake.setIntakeMode(hasBall);
//                break;
//            case OUTTAKE:
//                robot.intakeUptake.setOuttakeMode();
//                break;
//            case FEED:
//                robot.intakeUptake.setFeedMode();
//                break;
//        }
//    }

    public ShooterState getShooterState(){
        return shooterState;
    }

    public void updateShooter(boolean hasBall, boolean hadBall){
        switch (shooterState) {
            case SPEEDING_UP:
                robot.shooter.shooterTask();
                if (robot.shooter.isShooterReady(Shooter.Params.SHOOTER_TOLERANCE_RPM, Shooter.Params.PITCH_TOLERANCE, Shooter.Params.TURRET_TOLERANCE)){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                robot.shooter.shooterTask();
                setTransferState(TransferState.FEED);
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
            case WAITING:
                robot.shooter.shooterTask();
                if (shooterTimer.getElapsedTimeSeconds() > 0.6){
                    setShooterState(ShooterState.SPEEDING_UP);
                }
                break;
            case DONE:
                shotCounter = 0;
                robot.shooter.stopShooterSystem();
                break;

        }
    }


//
    public void update(boolean hasBall, boolean hadBall){
        updateShooter(hasBall, hadBall);
        robot.intakeUptake.intakeUptakeTask();
//        updateTurret();
    }


}
