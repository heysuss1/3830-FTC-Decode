package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Turret {
    public Servo pitchServo;

    public DcMotorEx turretMotor;
    public double servoRotations = 0;

    public kinematics kinematics = new kinematics();

    public double yawRaw;
    public int yaw;
    AnalogInput analogInput;
    public double voltToDegree(double volt){
        return (volt-0.5) * (360.0/4.0);
    }

    public double getDegrees(){
        double volt = analogInput.getVoltage();
        return (volt-0.5) * (360.0/4.0);
    }
    public double getServoVel(){
        return analogInput.getVoltage();
    }
    public void updateRotations(int rotations){
        servoRotations = rotations;
    }

    public void init(HardwareMap hwMap){
        pitchServo = hwMap.get(Servo.class, "pitchServo");
        turretMotor = hwMap.get(DcMotorEx.class,"turretRotation");
        analogInput = hwMap.get(AnalogInput.class, "analogInput");
    }

    public double ticksToDegrees(int ticks){
        double turretConversion = 6.7; //Change!!!
        return ticks * turretConversion;
    }

    public void setYaw(Follower follower) {
        yawRaw = kinematics.getYaw(
                follower.getPose().getX(),
                follower.getPose().getY(),
                Constants.getTEAM()
        );
        yaw = 67; //(calculation)

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //TODO: calculate yawRaw to target position
        turretMotor.setTargetPosition(yaw);// + calculation
    }

}
