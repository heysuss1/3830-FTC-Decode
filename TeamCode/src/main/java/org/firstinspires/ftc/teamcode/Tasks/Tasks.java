package org.firstinspires.ftc.teamcode.Tasks;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.PIDControls.TurretController;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Tasks {

    private Timer shooterTimer;
    private Hardware robot;
    private int shotCounter;


    public Tasks(Hardware robot){
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

    public enum TransferState{
        INTAKE,
        FEED,
        OUTTAKE,
        OFF
    }

    ShooterState shooterState = ShooterState.DONE;
    TransferState transferState = TransferState.OFF;

    public void setShooterState(ShooterState state){
        shooterState = state;
        shooterTimer.resetTimer();
    }


    public void setTransferState(TransferState state){
        transferState = state;
    }


    public void updateTransfer(boolean hasBall){
        switch (transferState){
            case OFF:
                robot.transfer.stopTransfer();
                break;
            case INTAKE:
                robot.transfer.setIntakeMode(hasBall);
                break;
            case OUTTAKE:
                robot.transfer.setOuttakeMode();
                break;
            case FEED:
                robot.transfer.setFeedMode();
                break;
        }
    }



    public void updateShooter(){
        switch (shooterState) {
            case SPEEDING_UP:
                robot.shooter.setPower(VelocityController.getPower(robot.shooter.getVelocity(), robot.shooter.getVelocityTarget()));
                if (robot.shooter.isReady(150)){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                setTransferState(TransferState.FEED);
                if (shooterTimer.getElapsedTimeSeconds() > 0.28) {
                    setTransferState(TransferState.OFF);
                    shotCounter++;
                    setShooterState(ShooterState.WAITING);
                    return;
                }

                if (shotCounter >= 3) {
                    setTransferState(TransferState.OFF);
                    setShooterState(ShooterState.DONE);
                }
                break;
            case WAITING:
                if (shooterTimer.getElapsedTimeSeconds() > 0.6){
                    setShooterState(ShooterState.SPEEDING_UP);
                }
                break;
            case DONE:
                robot.shooter.stopShooter();
                break;

        }
    }

    public void turretUpdate(){
        int currentTurretPosition = robot.shooter.getTurretPosition();
        int turretTargetPosition = robot.shooter.getTurretTargetPos();
        robot.shooter.setTurretPower(TurretController.getPower(currentTurretPosition, turretTargetPosition));
    }


}
