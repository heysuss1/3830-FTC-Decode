package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Turret {
    public Servo pitchServo;

    public DcMotorEx turretMotor;

    public kinematics kinematics = new kinematics();

    public double yawRaw;
    public int yaw;

    public void init(HardwareMap hwMap){
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        turretMotor = hwMap.get(DcMotorEx.class,"turretMotor");
    }

    public double ticksToDegrees(int ticks){
        double turretConversion = 6.7; //Change!!!
        return ticks * turretConversion;
    }

    public void setYaw(Follower follower) {
        yawRaw = kinematics.getYaw(
                follower.getPose().getX(),
                follower.getPose().getY(),
                RobotConstants.getTEAM()
        );
        yaw = 67; //(calculation)

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //TODO: calculate yawRaw to target position
        turretMotor.setTargetPosition(yaw);// + calculation
    }

}
