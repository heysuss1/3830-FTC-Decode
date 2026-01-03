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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.controllers.PidfController;

public class Shooter {
    //TODO: add shooter motors/servos

    //1 shooter motor, 1 servo for pitch, 1 motor to pan/turret

    public static final class Params {


        //Shooter Params

        public static final double GOBUILDA_5000_CPR = 28.0;
        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1;
        public static final double SHOOTER_TICKS_PER_REV = GOBUILDA_5000_CPR * SHOOTER_MOTOR_GEAR_RATIO;//update gear ratio
        public static final double SHOOTER_KP = (((67))), SHOOTER_KI = (((67))), SHOOTER_KD = (((67))), SHOOTER_KF = (((67))), SHOOTER_I_ZONE = (((67)));
        public static final int SHOOTER_TOLERANCE_RPM = (((100)));

        //Pitch Params

        public static final int MIN_PITCH_DEGREES = 25;
        public static final int MAX_PITCH_DEGREES = 50;
        public static final int PITCH_GEAR_RATIO = (15/173) * (48/20); //.208

        public static  final double PITCH_ENCODER_ZERO_OFFSET = 0;
        public static  final double PITCH_POSITION_OFFSET = MIN_PITCH_DEGREES;
        public static final double PITCH_DEGREES_PER_REV = 360 * PITCH_GEAR_RATIO;




        //Turret Params

        public static final double TURRET_KP = (((67))), TURRET_KI = (((67))), TURRET_KD = (((67))), TURRET_KF = (((67))), TURRET_I_ZONE = (((67)));
        public static final int TURRET_TICKS_PER_REV = 67;
        public static final int TURRET_GEAR_RATIO = 41/88; //Servo gear / turret gear

        public static final int MIN_TURRET_DEGREES = -90;
        public static final int MAX_TURRET_DEGREES = 90;

        public static  final double TURRET_ENCODER_ZERO_OFFSET = 0;
        public static  final double TURRET_POSITION_OFFSET = 0;
        public static final double TURRET_DEGREES_PER_REV = 360 * TURRET_GEAR_RATIO;


    }

    private final DcMotorEx bottomShooterMotor;
    private final DcMotorEx topShooterMotor;

    private final Servo pitchServo;
    private final AnalogInput pitchEncoder;

    private final CRServo primaryTurretServo;
    private final CRServo secondaryTurretServo;
    private final AnalogInput turretEncoder;

    private final Telemetry telemetry;
    private final PidfController shooterController;
    private final PidfController turretController;

    private final ElapsedTime timer;
    public final Hardware robot;

    public Double velocityTarget = null;
    public Double pitchTarget = null;
    public Double turretTarget = null;

    private double timeout = 0.0;

    public static boolean alwaysSetVelocity = false;
    public static boolean alwaysAimShooter = false;

    public Shooter(HardwareMap hwMap, Telemetry telemetry, Hardware robot) {

        this.telemetry = telemetry;
        this.robot = robot;
        timer = new ElapsedTime();

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
        turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");

        shooterController = new PidfController(Params.SHOOTER_KP, Params.SHOOTER_KI, Params.SHOOTER_KD, Params.SHOOTER_KF, Params.SHOOTER_I_ZONE);
        turretController = new PidfController(Params.TURRET_KP, Params.TURRET_KI, Params.TURRET_KD, Params.TURRET_KF, Params.TURRET_I_ZONE);
    }

    public PidfController getShooterController() {
        return shooterController;
    }
    public PidfController getTurretController() {
        return turretController;
    }

    public Double getVelocityRPM() {
        return (Math.abs(topShooterMotor.getVelocity()) * 60)/Params.SHOOTER_TICKS_PER_REV;
    }

    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / 3.3);
        double encoderOffset = (rawPitchPos - Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * Params.PITCH_DEGREES_PER_REV) + Params.PITCH_POSITION_OFFSET;
    }
    
    public double getTurretDegrees() {
        return 0;

        /*
        double rawPitchPos = robot.shooter.getTurretRawPose();
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        double currentDegrees = (encoderOffset * Shooter.Params.TURRET_DEGREES_PER_REV) + Shooter.Params.PITCH_POSITION_OFFSET;
        return currentDegrees + crossovers * Shooter.Params.TURRET_DEGREES_PER_REV;
         */

        //TODO: test in the TurretTester class
    }

    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/3.3;
    }

    public void setVelocityTarget(Double velocityTargetRPM, double timeout) {
        this.velocityTarget = velocityTargetRPM;
        this.timeout = timeout;
    }

    public void setVelocityTarget(Double velocityTargetRPM) {
        setVelocityTarget(velocityTargetRPM, 0.0);
    }

    public void setPitchDegrees(Double targetPitchDegrees) {
        double targetAngle = Range.clip(targetPitchDegrees, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
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
        return Math.abs(velocityTarget - getVelocityRPM()) <= tolerance;
    }


    public double getTurretPosition(){
        return ticksToDegreesTurret(bottomShooterMotor.getCurrentPosition()) ;
    }

    public int getTurretTargetPos(){
        return degreesToTicks(getYaw(robot.follower.getPose()));
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
            double currentVelocity = getVelocityRPM();
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

