package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Turret {
    public Servo pitchServo;

    public DcMotorEx turretMotor;
    public double servoRotations = 0;

    public double gearRatio = 0.3819;

    public final double TICKS_PER_REV = 142.8;

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
        turretMotor = hwMap.get(DcMotorEx.class,"turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setPower(0);
    }

    public int degreesToTicks(double degrees){
        int ticks = (int)Math.round((degrees/(360.0 * gearRatio)) * TICKS_PER_REV); //Change!!!
        return ticks;
    }

    public double setYaw(Follower follower) {
        yawRaw = kinematics.getYaw(
                follower.getPose().getX(),
                follower.getPose().getY(),
                RobotConstants.getTEAM()
        );
        return yawRaw;


    }

}
