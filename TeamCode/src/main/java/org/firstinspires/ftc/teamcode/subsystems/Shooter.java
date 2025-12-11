package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret
    public kinematics kinematics = new kinematics();
    public DcMotorEx topShooterMotor;
    public DcMotorEx bottomShooterMotor;
    public static final int TICKS_PER_REVOLUTION = 28;
    public static final int SHOT_POS_VEL = 3500;
    public Servo pitchServo;
    public double pitchRaw;

    public enum Shooter_state{
        OFF,
        SPEEDING_UP,
        WAITING_FULL_SPEED,
        SLOWING_DOWN,
    }

    // hooray

    public Shooter(HardwareMap hwMap){

//        pitchServo = hwMap.get(Servo.class, "pitchServo");

        topShooterMotor = hwMap.get(DcMotorEx.class, "topShooterMotor");
        topShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topShooterMotor.setPower(0);

        bottomShooterMotor = hwMap.get(DcMotorEx.class, "bottomShooterMotor");
        bottomShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooterMotor.setPower(0);

    }
    public boolean isReady(int velTarget, int tolerance){
        return Math.abs(velTarget - getVelocity()) <= tolerance;
    }

    public void stopShooter(){
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPower(0);
    }


    /*
    Takes in rpm, converts it to ticks per second, and passes it into the function.
     */
    public void setVelocity(int velocity){
        topShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topShooterMotor.setVelocity(RPMtoTPS(velocity));
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomShooterMotor.setVelocity(RPMtoTPS(velocity));
        
        
    }

    public double RPMtoTPS(int rpm){
        return (rpm*TICKS_PER_REVOLUTION/60);
    }
    public double TPStoRPM(double tps) {return (60*tps/TICKS_PER_REVOLUTION);}



    public void setPitchServo(Follower follower) {
        pitchRaw = kinematics.getPitch(
                follower.getPose().getX() + RobotConstants.ballXOffset,
                follower.getPose().getY() + RobotConstants.ballYOffset,
                536,
                false,
                RobotConstants.getTEAM()
        );
        //TODO: add calculation for converting pitchRaw to servo position, prob linear
        pitchServo.setPosition(pitchRaw); //+ calculation
    }


    //IN RPM
    public double getVelocity() { return (topShooterMotor.getVelocity() * 60)/TICKS_PER_REVOLUTION; }

    public void setPower(double power){
        topShooterMotor.setPower(power);
    }
    public void prepShooter(){
        setVelocity(SHOT_POS_VEL);
    }

}

