package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret
    public kinematics kinematics = new kinematics();
    public DcMotorEx shootingMotor;
    public CRServo uptakeServo;
    public Timer uptakeTimer;
    public Servo pitchServo;
    public DcMotorEx turretRotation;
    public double pitchRaw;

    // hooray

    public void init(HardwareMap hwMap){
//        pitchServo = hwMap.get(Servo.class, "pitchServo");
        shootingMotor = hwMap.get(DcMotorEx.class, "shootingMotor");
        uptakeServo = hwMap.get(CRServo.class, "uptakeServo");
         uptakeTimer = new Timer();

    }

    public void setPitchServo(Follower follower) {
        pitchRaw = kinematics.getPitch(
                follower.getPose().getX() + Constants.ballXOffset,
                follower.getPose().getY() + Constants.ballYOffset,
                Constants.getTEAM()
        );
        //TODO: add calculation for converting pitchRaw to servo position, prob linear
        pitchServo.setPosition(pitchRaw); //+ calculation
    }


    public void startUptake(){
        uptakeServo.setPower(1);
    }

    public void stopUptake(){
        uptakeServo.setPower(0);
    }

    public double getVelocity() { return shootingMotor.getVelocity(); }



    public void setPower(double power){
        shootingMotor.setPower(power);
    }

}

