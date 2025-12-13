package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret
    public kinematics kinematics = new kinematics();
    public DcMotorEx topShooterMotor;
    public final double HAS_BALL_TRESHOLD = 1.7;
    public DcMotorEx bottomShooterMotor;

    public ShotLocation[] lookUpTable = {
            new ShotLocation(30, 100)
    };
    public double currentVel;
    public double prevVel;
    public static final int TICKS_PER_REVOLUTION = 28;
    public static final int SHOT_POS_VEL = 3500;
    public Servo pitchServo;
    public double pitchRaw;


    public RevColorSensorV3 colorSensor;
    public enum Shooter_state{
        OFF,
        SPEEDING_UP,
        WAITING_FULL_SPEED,
        SLOWING_DOWN,
    }

    // hooray

    public Shooter(HardwareMap hwMap){

//        pitchServo = hwMap.get(Servo.class, "pitchServo");

        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");

        topShooterMotor = hwMap.get(DcMotorEx.class, "topShooterMotor");
        topShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        topShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        topShooterMotor.setPower(0);

        bottomShooterMotor = hwMap.get(DcMotorEx.class, "bottomShooterMotor");
        bottomShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bottomShooterMotor.setPower(0);

    }
    public boolean isReady(int velTarget, int tolerance){
        return Math.abs(velTarget - getVelocity()) <= tolerance;
    }

    public boolean hasBall(){
            if (colorSensor.getDistance(DistanceUnit.INCH) < HAS_BALL_TRESHOLD){
            return true;
        } else {
            return false;
        }
    }

    public void getClosestRPM(double distance){
        int closestRPM = lookUpTable[0].rpm;
        double closestDistance = Math.abs(lookUpTable[0].distance - distance);
        for (int i = 0; i < lookUpTable.length; i++){
            double shotDistance =  Math.abs(distance - lookUpTable[i].distance);
            if (shotDistance < closestDistance){
                closestDistance = shotDistance;
                closestRPM = lookUpTable[i].rpm;
            }
        }
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
        return (rpm*TICKS_PER_REVOLUTION/60.0);
    }

    public boolean isBallShot(int prevVel, int currentVel){
        if (Math.abs(currentVel - prevVel) > 200){
            return true;
        }
        return false;
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
    public double getVelocity() { return (Math.abs(topShooterMotor.getVelocity()) * 60)/TICKS_PER_REVOLUTION; }

    public void setPower(double power){
        topShooterMotor.setPower(power);
        bottomShooterMotor.setPower(topShooterMotor.getPower());
    }
    public void prepShooter(){
        setVelocity(SHOT_POS_VEL);
    }

}

