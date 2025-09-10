package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class BlueSideAuto extends OpMode {



    Follower follower;
    Hardware robot = Hardware.getInstance();
    boolean startAuto = false;

    //TODO: find actual headings.

    Pose startingPose = new Pose(56, 8, 0);
    Pose shootingPosition = new Pose(54.9, 108, 145);
    Pose ball1Pose = new Pose(45, 84, 90);
    Pose ball2Pose = new Pose(45, 60, 90);
    Pose ball3Pose = new Pose(45, 36, 90);
    Pose parkPose = new Pose(39, 33, 0);
    Pose gatePose = new Pose(18, 69);

    //Scan for the motif and dont move until the motif has been verified.
    PathChain toDepot, toBall1, toDepotFromBall1, toBall2, toDepotFromBall2, toBall3, toDepotfromBall3, toParking;

//    public void

    public void buildPaths(){
        toDepot = follower.pathBuilder()
                .addPath( new BezierLine(startingPose, shootingPosition))
                .setLinearHeadingInterpolation(0, shootingPosition.getHeading())
                .build();
        toBall1 = follower.pathBuilder()
                .addPath( new BezierLine(shootingPosition, ball1Pose))
                .setLinearHeadingInterpolation(shootingPosition.getHeading(), ball1Pose.getHeading())
                .build();
        toDepotFromBall1 = follower.pathBuilder()
                .addPath( new BezierLine(ball1Pose, shootingPosition))
                .setLinearHeadingInterpolation(ball1Pose.getHeading(), shootingPosition.getHeading())
                .build();
        toBall2 = follower.pathBuilder()
                .addPath( new BezierLine(shootingPosition, ball2Pose))
                .setLinearHeadingInterpolation(shootingPosition.getHeading(), ball2Pose.getHeading())
                .build();
        toDepotFromBall2 = follower.pathBuilder()
                .addPath( new BezierLine(ball2Pose, shootingPosition))
                .setLinearHeadingInterpolation(ball2Pose.getHeading(), shootingPosition.getHeading())
                .build();
        toBall3 = follower.pathBuilder()
                .addPath( new BezierLine(shootingPosition, ball3Pose))
                .setLinearHeadingInterpolation(shootingPosition.getHeading(), ball3Pose.getHeading())
                .build();
        toDepotfromBall3 = follower.pathBuilder()
                .addPath( new BezierLine(ball3Pose, shootingPosition))
                .setLinearHeadingInterpolation(ball3Pose.getHeading(), shootingPosition.getHeading())
                .build();
        toParking = follower.pathBuilder()
                .addPath( new BezierLine(shootingPosition, parkPose))
                .setLinearHeadingInterpolation(shootingPosition.getHeading(), parkPose.getHeading())
                .build();



    }


    public void init(){
//        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);
        robot.init(hardwareMap);
        buildPaths();
    }
    public void init_loop() {
        if (gamepad1.dpad_left) {
            Constants.setTeam(Constants.Team.BLUE);
        }
        if (gamepad1.dpad_right) {
            Constants.setTeam(Constants.Team.RED);
        }

        telemetry.addData("Team Color: ", Constants.getTEAM());
        telemetry.update();
    }

    public void pathUpdate(){

    }
    public void actionUpdate(){

    }
    public void loop(){
        if (robot.vision.getMotif() != Constants.Motif.NULL){
            Constants.setMotif(robot.vision.getMotif());
            startAuto = true;
        }

        if (startAuto){
            pathUpdate();
            actionUpdate();
        }
        follower.update();
        telemetry.update();
    }

}
