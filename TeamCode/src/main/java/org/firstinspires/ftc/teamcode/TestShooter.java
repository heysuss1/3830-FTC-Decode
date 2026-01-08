package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.X_GOAL_BLUE;
import static org.firstinspires.ftc.teamcode.Robot.X_GOAL_RED;
import static org.firstinspires.ftc.teamcode.Robot.Y_GOAL;

import org.firstinspires.ftc.teamcode.subsystems.ShootParams;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.PidfController;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TestShooter {

    public class AimInfo {
        public double distance;
        public double angle;

        public AimInfo(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }

        public double getDistanceToGoal() {
            return distance;
        }

        public double getAngleToGoal() {
            return angle;
        }
    }

    public static final ShootParams.Region[] shootRegions = {
            //Region 1: pitch 25 degrees, y = 3200 + 15x
            new ShootParams.Region(25.0, new double[][] {{3200.0, 15.0}})
    };

    public static final ShootParams shootParamsTable = new ShootParams()
            .addEntry("target_2ft", 24, shootRegions[0], 3500);

    public static final class Params {

        //Shooter Params

        public static final double GOBUILDA_5000_CPR = 28.0;
        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1;
        public static final double SHOOTER_TICKS_PER_REV = GOBUILDA_5000_CPR * SHOOTER_MOTOR_GEAR_RATIO;//update gear ratio
        public static final double SHOOTER_KP = 0.001, SHOOTER_KI = 0, SHOOTER_KD = 0, SHOOTER_KF = 0.00024, SHOOTER_I_ZONE = 150;
        public static final int SHOOTER_TOLERANCE_RPM = (((100)));

        //Pitch Params

        public static final double MIN_PITCH_DEGREES = 26; //29
        public static final double MAX_PITCH_DEGREES = 53;
        public static final double PITCH_GEAR_RATIO = .177 ; //.208 //(15.0/173) * (48.0/20)
        public static  final double PITCH_ENCODER_ZERO_OFFSET = 0.49; //0.48
        public static  final double PITCH_POSITION_OFFSET = MIN_PITCH_DEGREES;
        public static final double PITCH_DEGREES_PER_REV = 360 * PITCH_GEAR_RATIO; //360 * PITCH_GEAR_RATIO
        public static final double PITCH_TOLERANCE = 1;

        //Turret Params

//        PIDFController pidf = new PIDFController(0.0025, 0, 0, 0.000246, 100);
        public static final double TURRET_KP = (((67))), TURRET_KI = (((67))), TURRET_KD = (((67))), TURRET_KF = (((67))), TURRET_I_ZONE = (((67)));
        public static final double TURRET_TICKS_PER_REV = 67;
        public static final double TURRET_GEAR_RATIO = 0.535; //43.0/88 //Servo gear / turret gear

        public static final double MIN_TURRET_DEGREES = -90;
        public static final double MAX_TURRET_DEGREES = 90;

        public static final double TURRET_ENCODER_ZERO_OFFSET = 0;
        public static final double TURRET_POSITION_OFFSET = 0;
        public static final double TURRET_DEGREES_PER_REV = 360 * TURRET_GEAR_RATIO;
        public static final double TURRET_TOLERANCE = 5;

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
    public Double velocityTarget = null;
    public Double pitchTarget = null;
    public Double turretTarget = null;
    public Double currentShooterVelTarget = null;
    public Follower follower;


    private double timeout = 0.0;

    public static boolean alwaysSetVelocity = false;
    public static boolean alwaysAimShooter = false;

    public TestShooter(HardwareMap hwMap, Telemetry telemetry, Follower follower) {

        this.telemetry = telemetry;
        timer = new ElapsedTime();
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

    public Double getVelocityTarget(){
        return velocityTarget;
    }

    public Double getVelocityRPM() {
        return (Math.abs(topShooterMotor.getVelocity()) * 60)/Params.SHOOTER_TICKS_PER_REV;
    }

    public AnalogInput getPitchEncoder() {
        return pitchEncoder;
    }

    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / pitchEncoder.getMaxVoltage());
        return rawPitchToDegrees(rawPitchPos);
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * Params.PITCH_DEGREES_PER_REV) + Params.PITCH_POSITION_OFFSET;
    }

    public double degreesToRawPitch(double degrees) {
        double pitchZeroOffset = (degrees - Params.PITCH_POSITION_OFFSET) / (Params.PITCH_GEAR_RATIO * 360);
        return pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
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


    public DcMotorEx getShooterMotor(){
        return topShooterMotor;
    }

    public CRServo getPrimaryTurretServo(){
        return primaryTurretServo;
    }
    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/turretEncoder.getMaxVoltage();
    }

    public Double getCurrentShooterVelTarget(){
        return currentShooterVelTarget;
    }

    public void setVelocityTarget(Double velocityTargetRPM, double timeout) {
        this.velocityTarget = velocityTargetRPM;
        this.timeout = timeout;
    }

    public void setVelocityTarget(Double velocityTargetRPM) {
        setVelocityTarget(velocityTargetRPM, 0.0);
    }

    public void setPitchDegrees(Double targetPitchDegrees) {
//        double targetAngle = Range.clip(targetPitchDegrees, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
        pitchTarget = degreesToRawPitch(targetPitchDegrees);
    }

    public void setTurretDegrees(Double targetDegrees) {
        turretTarget = targetDegrees;
    }

    public void setTurretDegrees(AimInfo aimInfo) {
        double turretTargetRaw = aimInfo.getAngleToGoal() - follower.getHeading();
        double turretTargetModulo = (turretTargetRaw + 180) % 360 - 180;
        //once it can rotate more, add some code to modulo this btwn -180 and 180 (like (n-180)%360+180 or smth)
        turretTarget = Range.clip(turretTargetModulo, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
    }
    public void resetTimeout() {
        timeout = 0.0;
    }

    public void stopShooterSystem() {
        stopShooterMotor();
        stopPitchServo();
        stopTurret();
    }

    public void stopShooterMotor() {
        if (velocityTarget != null) {
            velocityTarget = null;
            resetTimeout();
            topShooterMotor.setPower(0);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
    }

    public void stopPitchServo() {
        if (pitchTarget != null) {
            pitchTarget = null;
        }
    }

    public AimInfo getAimInfo(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double deltaX = Robot.getTEAM() == Robot.Team.BLUE ? robotX - X_GOAL_BLUE: robotX - X_GOAL_RED;
        double deltaY = robotY - Y_GOAL;
        double distanceToGoal = Math.hypot(deltaY, deltaX);
        double angleToGOal = Math.toDegrees(Math.atan2(deltaY, deltaX)) + 180;

        return new AimInfo(distanceToGoal, angleToGOal);
    }

    public void stopTurret() {
        if (turretTarget != null) {
            turretTarget = null;
            primaryTurretServo.setPower(0);
            secondaryTurretServo.setPower(0);
        };
    }

    public boolean isFlywheelOnTarget(int tolerance) {
        boolean isOnTarget = false;
        if (velocityTarget != null) {
            isOnTarget = ((timeout > 0.0) && (timer.seconds() >= timeout)) || (Math.abs(velocityTarget - getVelocityRPM()) <= tolerance);
                            //timer expired                                 or                  rpm is good
        }
        return isOnTarget;
    }

    public boolean isPitchOnTarget(int tolerance) {
        boolean isOnTarget = false;
        if (pitchTarget != null) {
            isOnTarget = Math.abs(rawPitchToDegrees(pitchTarget) - getPitchDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isTurretOnTarget(int tolerance) {
        boolean isOnTarget = false;
        if (turretTarget != null) {
            isOnTarget = Math.abs(turretTarget - getTurretDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isShooterReady(int flywheelTolerance, int pitchTolerance, int turretTolerance) {
        return (velocityTarget == null || isFlywheelOnTarget(flywheelTolerance))
                && (pitchTarget == null || isPitchOnTarget(pitchTolerance))
                && (turretTarget == null || isTurretOnTarget(turretTolerance));
    }

    public void flywheelTask() {
        double output = 0;
        if (velocityTarget != null) {
            double currentVelocity = getVelocityRPM();
            output = shooterController.calculate(velocityTarget, currentVelocity);
            topShooterMotor.setPower(output);
            bottomShooterMotor.setPower(topShooterMotor.getPower());
        }
        if (!Robot.inComp) {
            telemetry.addLine("\nFlywheel Info:");
            telemetry.addData("Target Velocity (rpm)", velocityTarget );
            telemetry.addData("Current Velocity (rpm)", getVelocityRPM());
            telemetry.addData( "Power", output);
            telemetry.addData("Error", shooterController.getError());
            telemetry.update();
        }
    }

    public void pitchTask() {
        if (pitchTarget != null) {
            pitchServo.setPosition((1-degreesToRawPitch((rawPitchToDegrees(pitchTarget)-3)/0.888)));
        }
        if (!Robot.inComp) {
            telemetry.addLine("\nPitch Info:");
            telemetry.addData("Pitch Target", pitchTarget);
            telemetry.addData("Current Pitch Degrees", getPitchDegrees());
            telemetry.addData("Curent Pitch Raw", (pitchEncoder.getVoltage() / 3.3));
            telemetry.update();
        }
    }

    public void turretTask() {
        double output = 0;
        if (turretTarget != null) {
            double currentPosition = getTurretDegrees();
            output = turretController.calculate(turretTarget, currentPosition);
            primaryTurretServo.setPower(output);
            secondaryTurretServo.setPower(output);
        }

        if (!Robot.inComp) {
            telemetry.addLine("\nTurret Info:");
            telemetry.addData("Turret Target", turretTarget);
            telemetry.addData("Current Turret Degrees", getTurretDegrees());
            telemetry.addData( "Power", output);
            telemetry.addData("Error", turretController.getError());
            telemetry.update();
        }
    }

    public void shooterTask() {
        flywheelTask();
        pitchTask();
        turretTask();

        AimInfo aimInfo = getAimInfo();
        ShootParams.Entry shootParams = Shooter.shootParamsTable.get(aimInfo.getDistanceToGoal());
//        setPitchDegrees(shootParams.region.tiltAngle);
        setTurretDegrees(aimInfo);

        currentShooterVelTarget = shootParams.outputs[0];
        if (alwaysSetVelocity) {
            setVelocityTarget(shootParams.outputs[0]);
        }
    }
}


