package org.firstinspires.ftc.teamcode.subsystems;

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
import org.firstinspires.ftc.teamcode.TestShooter;
import org.firstinspires.ftc.teamcode.controllers.PidfController;

public class Shooter {


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
        public static final double SHOOTER_KP = 0.001, SHOOTER_KI = 0.01, SHOOTER_KD = 0, SHOOTER_KF = 0.0002, SHOOTER_I_ZONE = 150;
        public static final double SHOOTER_TOLERANCE_RPM = (((100)));

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
        public static final double CROSSOVER_THRESHOLD = 0.5;
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
    public final Robot robot;

    public Double velocityTarget = null;
    public Double pitchTarget = null;
    public Double turretTarget = null;
    public Double currentShooterVelTarget = null;


    public double currentVoltage = 0;
    public double prevVoltage = 0;
    private int crossovers = 0;

    private double timeout = 0.0;

    public static boolean alwaysSetVelocity = false;
    public static boolean alwaysAimShooter = false;

    public Shooter(HardwareMap hwMap, Telemetry telemetry, Robot robot) {

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

    public Double getVelocityTarget(){
        return velocityTarget;
    }

    public Double getVelocityRPM() {
        return (Math.abs(topShooterMotor.getVelocity()) * 60)/Params.SHOOTER_TICKS_PER_REV;
    }

    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / 3.3);
        return rawPitchToDegrees(rawPitchPos);
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * (Shooter.Params.PITCH_GEAR_RATIO * 360)) + Shooter.Params.PITCH_POSITION_OFFSET;
    }

    public double getTurretDegrees() {
        double rawPitchPos = (1-getTurretRawPose()) + 0.987 * crossovers;
        double encoderOffset = (rawPitchPos - TestShooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        double unconverted = (encoderOffset * TestShooter.Params.TURRET_DEGREES_PER_REV) + TestShooter.Params.PITCH_POSITION_OFFSET;
        return modularConversion(unconverted);
    }

    public int getCrossovers(){
        return crossovers;
    }
    public void countCrossovers() {

        prevVoltage = currentVoltage;
        currentVoltage = getTurretRawPose();

        if (Math.abs(currentVoltage - prevVoltage) > Params.CROSSOVER_THRESHOLD
        ) {
            if (currentVoltage > prevVoltage) {
                crossovers--;
            }
            else if (currentVoltage < prevVoltage) {
                crossovers++;
            }
        }
    }

    public double modularConversion(double n) {
        if (n >= 0) {
            return (n + 180) % 360 - 180;
        } else {
            return -modularConversion(-n);
        }
    }



    public DcMotorEx getShooterMotor(){
        return topShooterMotor;
    }

    public CRServo getPrimaryTurretServo(){
        return primaryTurretServo;
    }
    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/3.3;
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
        double targetAngle = Range.clip(targetPitchDegrees, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
        double pitchZeroOffset = (targetAngle - Params.PITCH_ENCODER_ZERO_OFFSET) / Params.PITCH_DEGREES_PER_REV;
        pitchTarget = pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
    }
    public void setTurretDegrees(Double targetDegrees) {
        turretTarget = targetDegrees;
    }

    public void setTurretDegrees(Robot.AimInfo aimInfo) {
        double turretTargetRaw = aimInfo.getAngleToGoal() - robot.follower.getHeading();
        double turretTargetModulo = modularConversion(turretTargetRaw);
        //once it can rotate more, add some code to modulo this btwn -180 and 180 (like (n-180)%360+180 or smth)
        turretTarget = turretTargetModulo;
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

    public void stopTurret() {
        if (turretTarget != null) {
            turretTarget = null;
            primaryTurretServo.setPower(0);
            secondaryTurretServo.setPower(0);
        };
    }

    public double degreesToRawPitch(double degrees) {
        double pitchZeroOffset = (degrees - Params.PITCH_POSITION_OFFSET) / (Params.PITCH_GEAR_RATIO * 360);
        return pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
    }


    public boolean isFlywheelOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (velocityTarget != null) {
            isOnTarget = ((timeout > 0.0) && (timer.seconds() >= timeout)) || (Math.abs(velocityTarget - getVelocityRPM()) <= tolerance);
                            //timer expired                                 or                  rpm is good
        }
        return isOnTarget;
    }

    public boolean isPitchOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (pitchTarget != null) {
            isOnTarget = Math.abs(rawPitchToDegrees(pitchTarget) - getPitchDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isTurretOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (turretTarget != null) {
            isOnTarget = Math.abs(turretTarget - getTurretDegrees()) <= tolerance;
        }
        return isOnTarget;
    }

    public boolean isShooterReady(double flywheelTolerance, double pitchTolerance, double turretTolerance) {
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
            telemetry.update();
        }
    }

    public void turretTask() {
        double output = 0;
        if (turretTarget != null) {
            countCrossovers();
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
        
        Robot.AimInfo aimInfo = robot.getAimInfo();
        ShootParams.Entry shootParams = Shooter.shootParamsTable.get(aimInfo.getDistanceToGoal());
        setPitchDegrees(shootParams.region.tiltAngle);
        setTurretDegrees(aimInfo);

        currentShooterVelTarget = shootParams.outputs[0];
        if (alwaysSetVelocity) {
            setVelocityTarget(shootParams.outputs[0]);
        }
    }
}
