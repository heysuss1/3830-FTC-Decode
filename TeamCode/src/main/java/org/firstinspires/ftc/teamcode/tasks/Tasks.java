package org.firstinspires.ftc.teamcode.tasks;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.zArchive.VelocityController;

public class Tasks {

    private Timer shooterTimer;
    private Timer ballCountTimer;
    private Hardware robot;
    private int shotCounter;
    private boolean isAuto;

    //TODO: refactor this class


    public Tasks(Hardware robot, HardwareMap hardwareMap, boolean isAuto){
        this.robot = robot;
        shooterTimer = new Timer();
        shotCounter = 0;
        ballCountTimer = new Timer();
        this.isAuto = isAuto;
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

    public TransferState getTransferState(){
        return transferState;
    }



    private void setShooterPower(){
        robot.shooter.setPower(velController.getPower(Math.abs(robot.shooter.getVelocity()), robot.shooter.getVelocityTarget()));

    }


    public void updateShooter(boolean hasBall, boolean hadBall){
        switch (shooterState) {
            case SPEEDING_UP:
                setShooterPower();
                if (robot.shooter.isReady(100)){
                    setShooterState(ShooterState.SHOOTING);
                }
                break;
            case SHOOTING:
                setShooterPower();
                setTransferState(TransferState.FEED);
                if (hadBall && !hasBall) {
                    setTransferState(TransferState.OFF);
                    shotCounter++;
                    setShooterState(ShooterState.WAITING);
                    return;
                }

                if (shotCounter >= 3) {
                    setTransferState(TransferState.OFF);
                    setShooterState(ShooterState.DONE);
                }
                if (shooterTimer.getElapsedTimeSeconds() > 1.5){
                    setShooterState(ShooterState.DONE);
                }
                break;
            case WAITING:
                setShooterPower();
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

//    public void updateTurret(){
//        int currentTurretPosition = robot.shooter.getTurretPosition();
//        int turretTargetPosition = robot.shooter.getTurretTargetPos();
//        robot.shooter.setTurretPower(TurretController.getPower(currentTurretPosition, turretTargetPosition));
//    }
//
    public void update(boolean hasBall, boolean hadBall){
        updateShooter(hasBall, hadBall);
        updateTransfer(hasBall);
//        updateTurret();
    }


}
