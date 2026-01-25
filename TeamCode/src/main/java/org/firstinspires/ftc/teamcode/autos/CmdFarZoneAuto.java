package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeUptake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.tasks.ShooterTask;
public class CmdFarZoneAuto{
//
//    enum AutoState {
//        START,
//        DRIVE_TO_SHOOTING_SPOT,
//        SHOOTING,
//        DRIVE_TO_LOADING_ZONE,
//        DRIVE_TO_PARK
//
//    }
//
//    AutoState autoState = AutoState.START;
//    Robot robot;
//    Timer pathTimer;
//    boolean isFirstTimePath;
//    final double AUTO_RPM = 4000;
//    int shotCount = 0;
//    @Override
//    public void buildPaths(){
//
//    }
//    @Override
//    public void autonomousUpdate(){
//        switch (autoState) {
//            case START:
//                //intentionally fall through!!111!11111111!!1
//            case DRIVE_TO_SHOOTING_SPOT:
//
//                robot.shooter.setVelocityTarget(AUTO_RPM);
//
//                if (shotCount == 0 && isFirstTimePath )
//                {
//                    robot.follower.followPath(driveToShootPreloads, true);
//                    isFirstTimePath = false;
//                }
//                if (shotCount == 1 && isFirstTimePath) { robot.follower.followPath(driveToShootGroup1, true); isFirstTimePath = false;}
//
//                if (!robot.follower.isBusy()) {
//                    isFirstTimePath = true;
//                    autoState = TwelveBallAuto.AutoState.SHOOTING;
//                }
//                break;
//            case SHOOTING:
//
//                if (isFirstTimePath){
//                    robot.shooterTask.startTask();
//                    isFirstTimePath = false;
//                }
//
//                if (robot.shooterTask.isFinished()) {
//                    isFirstTimePath = true;
//                    shotCount++;
//                    if (shotCount == 1) autoState = AutoState.DRIVE_TO_LOADING_ZONE;
//                    if (shotCount == 2) autoState = AutoState.DRIVE_TO_PARK;
//                }
//
//                break;
//            case DRIVE_TO_LOADING_ZONE:
//
//                break;
//
//        }
//    }
//    @Override
//    public void cancel(){
//
//    }
//    @Override
//    public String getAutoState(){
//        return autoState.toString();
//    }
//}
}