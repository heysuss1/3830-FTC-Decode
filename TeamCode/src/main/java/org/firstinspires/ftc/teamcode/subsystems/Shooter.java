package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDControls.TurretController;
import org.firstinspires.ftc.teamcode.PIDControls.VelocityController;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.kinematics;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret
    public kinematics kinematics = new kinematics();
    public final double HAS_BALL_TRESHOLD = 1.7;
    public double gearRatio = 0.3819;
    private final double TICKS_PER_REV = 142.8;
    private double xPosition;
    private double yPosition;
    private double headPosition;
    private int velocityTarget;
    private DcMotorEx turretMotor;
    private  DcMotorEx bottomShooterMotor;
    private DcMotorEx topShooterMotor;


    public ShotLocation[] lookUpTable = {
            new ShotLocation(30, 100)
    };
    public static final int TICKS_PER_REVOLUTION = 28;
    public Servo pitchServo;
    public double pitchRaw;


    public RevColorSensorV3 colorSensor;

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

////        turretMotor = hwMap.get(DcMotorEx.class,"turretMotor");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretMotor.setPower(0);

    }



    public double getTPSVelocity(){
        return topShooterMotor.getVelocity();
    }
    public void setRobotPose(double xPosition, double yPosition, double headPosition){
        this.xPosition = xPosition;
        this.yPosition = yPosition;
        this.headPosition = headPosition;
    }

    public int degreesToTicks(double degrees){
        int ticks = (int)Math.round((degrees/(360.0 * gearRatio)) * TICKS_PER_REV); //Change!!!
        return ticks;
    }

    public boolean isReady(int tolerance){
        return Math.abs(velocityTarget - getVelocity()) <= tolerance;
    }
    public boolean hasBall(){
            if (colorSensor.getDistance(DistanceUnit.INCH) < HAS_BALL_TRESHOLD){
            return true;
        } else {
            return false;
        }
    }

    public int getTurretPosition(){
        return turretMotor.getCurrentPosition();
    }

    public int getTurretTargetPos(){
        return degreesToTicks(setYaw());
    }
    public void setTurretPower(double power){
        turretMotor.setPower(power);
    }
    public double setYaw() {
        double yawRaw = kinematics.getYaw(
                xPosition,
                yPosition,
                RobotConstants.getTEAM()
        );
        return yawRaw - headPosition;
    }
    public int getVelocityTarget(){
        return velocityTarget;
    }
    public void setVelocityTarget(int velocityTarget) {this.velocityTarget = velocityTarget;}
    public int getClosestRPM(double distance){
        int closestRPM = lookUpTable[0].rpm;
        double closestDistance = Math.abs(lookUpTable[0].distance - distance);
        for (int i = 0; i < lookUpTable.length; i++){
            double shotDistance =  Math.abs(distance - lookUpTable[i].distance);
            if (shotDistance < closestDistance){
                closestDistance = shotDistance;
                closestRPM = lookUpTable[i].rpm;
            }
        }
        return closestRPM;
    }

    public void stopShooter(){
        setPower(0);
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
}

