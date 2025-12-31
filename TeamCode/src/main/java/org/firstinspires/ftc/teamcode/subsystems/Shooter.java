package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.RobotConstants.Y_GOAL;
import static org.firstinspires.ftc.teamcode.RobotConstants.inComp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.controllers.PidfController;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret

    public static final class Params {


        //Shooter Params

        private static final double shooterKP = (((67))), shooterKI = (((67))), shooterKD = (((67))), shooterKF = (((67))), shooterIZone = (((67)));
        public static final double HAS_BALL_TRESHOLD = 1.7;
        public static final double GEAR_RATIO = 0.3819;
        private static final double SHOOTER_TICKS_PER_REV = 28;

        //Pitch Params

        private static final int MIN_PITCH_DEGREES = 25;
        private static final int MAX_PITCH_DEGREES = 50;

        private  static  final double PITCH_ENCODER_ZERO_OFFSET = 0;
        private  static  final double PITCH_POSITION_OFFSET = 0;
        private static final double PITCH_DEGREES_PER_REV = 1;




        //Turret Params

        public static final int TURRET_TICKS_PER_REV = 67;

        public static final int TURRET_GEAR_RATIO = 67; //Servo gear / turret gear

    }

    private final DcMotorEx bottomShooterMotor;
    private final DcMotorEx topShooterMotor;

    private final Servo pitchServo;
    private final AnalogInput pitchEncoder;
    private final CRServo primaryTurretServo;
    private final CRServo secondaryTurretServo;

    private final Telemetry telemetry;
    private final PidfController shooterController;
    //TODO: figure out these constants ^^^


    public double pitchRaw;

    public Double velocityTarget = null;
    public Double pitchTarget = null;
    public Double turretTarget = null;

    public RevColorSensorV3 colorSensor;
    private Follower follower;

    // hooray

    public Shooter(HardwareMap hwMap, Follower follower, Telemetry telemetry){

//        pitchServo = hwMap.get(Servo.class, "pitchServo");
        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");

        this.telemetry = telemetry;
        this.follower = follower;

        pitchServo = hwMap.get(Servo.class, "pitchServo");
        pitchEncoder = hwMap.get(AnalogInput.class, "pitchEncoder");

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

        primaryTurretServo = hwMap.get(CRServo.class, "primaryTurretServo");
        secondaryTurretServo = hwMap.get(CRServo.class, "secondaryTurretServo");

        shooterController = new PidfController(Params.shooterKP, Params.shooterKI, Params.shooterKD, Params.shooterKF, Params.shooterIZone);

////        turretMotor = hwMap.get(DcMotorEx.class,"turretMotor");
//        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        turretMotor.setPower(0);
    }

    public PidfController getShooterController() {
        return shooterController;
    }

    public double getPitch(){
        double rawPitchPosition = (pitchEncoder.getVoltage()/3.3);
        double encoderOffset = (pitchTarget - Params.PITCH_ENCODER_ZERO_OFFSET);
        double pitchInDegrees = (encoderOffset * Params.PITCH_DEGREES_PER_REV) + Params.PITCH_POSITION_OFFSET;
        return pitchInDegrees;

    }

    public Double getVelocity() { return (Math.abs(topShooterMotor.getVelocity()) * 60)/Params.SHOOTER_TICKS_PER_REV; }
    public void setVelocityTarget(Double velocityTarget) {this.velocityTarget = velocityTarget;}


    public void setTiltPosition(Double targetTiltPosDeg)
    {
        double  targetAngle = Range.clip(targetTiltPosDeg, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
        double pitchZeroOffset = (targetAngle - Params.PITCH_ENCODER_ZERO_OFFSET) / Params.PITCH_DEGREES_PER_REV;
        pitchTarget = pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
    }
    public int degreesToTicks(double degrees){
        int ticks = (int)Math.round((degrees/(360.0 * Params.GEAR_RATIO)) * Params.SHOOTER_TICKS_PER_REV); //Change!!!
        return ticks;
    }

    public double ticksToDegreesTurret(double ticks){
        return Params.TURRET_GEAR_RATIO * 360 * ticks;
    }

    public boolean isReady(int tolerance){
        return Math.abs(velocityTarget - getVelocity()) <= tolerance;
    }
    public boolean hasBall(){
        if (colorSensor.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_TRESHOLD){
            return true;
        } else {
            return false;
        }
    }


    public double getTurretPosition(){
        return ticksToDegreesTurret(bottomShooterMotor.getCurrentPosition()) ;
    }

    public int getTurretTargetPos(){
        return degreesToTicks(getYaw(follower.getPose()));
    }

//    public void setTurretPower(double power){
//        turretMotor.setPower(power);
//    }
    public double getYaw(Pose robotPose){
        double x_goal =  RobotConstants.getTEAM() == RobotConstants.Team.BLUE ? RobotConstants.X_GOAL_BLUE : RobotConstants.X_GOAL_RED;
        return (Math.toDegrees(Math.atan2((robotPose.getY()-Y_GOAL), (robotPose.getX()-x_goal)))+180) - robotPose.getHeading();
    }

    public Double getVelocityTarget(){
        return velocityTarget;
    }


    public void stopShooter(){
        if (velocityTarget != null){
            velocityTarget = null;
            topShooterMotor.setPower(0);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
    }

    public boolean isBallShot(int prevVel, int currentVel){
        if (Math.abs(currentVel - prevVel) > 200){
            return true;
        }
        return false;
    }

    public void shooterTask() {
        if (velocityTarget != null) {
            double currentVelocity = getVelocity();
            double output = shooterController.calculate(velocityTarget, currentVelocity);
            topShooterMotor.setPower(output);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
            if (RobotConstants.inComp) {
                telemetry.addData("Target Velocity (rpm)",
                        velocityTarget );
                telemetry.addData("Current Velocity (rpm)", currentVelocity);
                telemetry.addData( "Power", output);
                telemetry.addData("Error", shooterController.getError());
                telemetry.update();
            }
        }
    }
}

