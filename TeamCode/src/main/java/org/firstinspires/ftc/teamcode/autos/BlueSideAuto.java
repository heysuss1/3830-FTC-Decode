package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

public class BlueSideAuto extends OpMode {



    Follower follower;
    Hardware robot = Hardware.getInstance();

    //TODO: find actual headings.
    Pose shootingPosition = new Pose(54.9, 108, 145);
    Pose ball1Pose = new Pose(45, 84, 90);
    Pose ball2Pose = new Pose(45, 60, 90);
    Pose ball3Pose = new Pose(45, 36, 90);
    Pose parkPose = new Pose(39, 33, 0);
    Pose gatePose = new Pose(18, 69);

    PathChain toDepot, toBall1, toDepotFromBall1, toBall2, toDepotFromBall2, toBall3, toDepotfromBall3, toParking;


//    public void



    public void init(){
//        follower = new Follower(hardwareMap);
        robot.init(hardwareMap);
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

    public void loop(){

    }

}
