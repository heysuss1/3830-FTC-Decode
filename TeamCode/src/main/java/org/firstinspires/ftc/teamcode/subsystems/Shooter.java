package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.controllers.PidfController;

public class Shooter {

    public static final ShootParams.Region[] shootRegions = {
            //Region 1: pitch 25 degrees, y = 3718.987 + 4.966x
            new ShootParams.Region(28.0, new double[][] {{3718.987, 4.966}}),
            //Region 2:
            new ShootParams.Region(30.0, new double[][] {{2899.984, 20}}),
            //Region 3:
            new ShootParams.Region(35.0, new double[][] {{2800, 20}}),

    };

    public static final ShootParams shootParamsTable = new ShootParams()
            .addEntry("target_50in", 49.9, shootRegions[0], 3950)
            .addEntry("target_60in", 59.9, shootRegions[0], 4050)
            .addEntry("target_70in", 70, shootRegions[0], 4050)
            .addEntry("target_70in", 70.00001, shootRegions[1], 4300)
            .addEntry("target_80in", 80, shootRegions[1], 4500)
            .addEntry("target_80in", 80.00001, shootRegions[2], 4500)
            .addEntry("target_90in", 90, shootRegions[2], 4500)
            .addEntry("target_100in", 100, shootRegions[2], 4700)
            .addEntry("target_110in", 110, shootRegions[2], 5050);

    public static final class Params {

        //Shooter Params
        public static final double GOBUILDA_5000_CPR = 28.0;
        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1;
        public static final double SHOOTER_TICKS_PER_REV = GOBUILDA_5000_CPR * SHOOTER_MOTOR_GEAR_RATIO;
        public static final double SHOOTER_KP = 0.001, SHOOTER_KI = 0.01, SHOOTER_KD = 0, SHOOTER_KF = 0.0002, SHOOTER_I_ZONE = 150;
        public static final double SHOOTER_TOLERANCE_RPM = 100;

        //Pitch Params
        public static final double MIN_PITCH_DEGREES = 26;
        public static final double MAX_PITCH_DEGREES = 53;
        public static final double PITCH_GEAR_RATIO = .177 ;
        public static  final double PITCH_ENCODER_ZERO_OFFSET = .5;
        public static  final double PITCH_POSITION_OFFSET = MIN_PITCH_DEGREES;
        public static final double PITCH_DEGREES_PER_REV = 360 * PITCH_GEAR_RATIO;
        public static final double PITCH_TOLERANCE = 2;
        public static final double VEL_DROP_PITCH_SCALE = 0;

        //Turret Params
        public static final double TURRET_KP = 0.016, TURRET_KI = (.01), TURRET_KD = (.00003), TURRET_KF = (0), TURRET_I_ZONE = (5);
        public static final double TURRET_GEAR_RATIO = 0.506;
        public static final double MIN_TURRET_DEGREES = -90;
        public static final double CROSSOVER_THRESHOLD = 0.5;
        public static final double MAX_TURRET_DEGREES = 90;
        public static final double TURRET_ENCODER_ZERO_OFFSET = 0;
        public static final double TURRET_POSITION_OFFSET = 0;
        public static final double TURRET_DEGREES_PER_REV = -360 * TURRET_GEAR_RATIO;
        public static final double TURRET_TOLERANCE = 3;
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

    public final Robot robot;

    public Double velocityTarget = null;
    public Double pitchTarget = null;
    public Double turretTarget = null;

    public double currentVoltage = 0;
    public double prevVoltage = 0;
    private int crossovers = 0;

    private static boolean alwaysSetVelocity = false;
    private static boolean alwaysAimTurret = false;
    private static boolean alwaysAimPitch = false;
    private static boolean compensateForVelDropWithPitch = false;

    public Shooter(HardwareMap hwMap, Telemetry telemetry, Robot robot) {

        this.telemetry = telemetry;
        this.robot = robot;

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
        secondaryTurretServo.setDirection(DcMotorSimple.Direction.FORWARD);
        turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");

        shooterController = new PidfController(Params.SHOOTER_KP, Params.SHOOTER_KI, Params.SHOOTER_KD, Params.SHOOTER_KF, Params.SHOOTER_I_ZONE);
        turretController = new PidfController(Params.TURRET_KP, Params.TURRET_KI, Params.TURRET_KD, Params.TURRET_KF, Params.TURRET_I_ZONE);
    }

    public boolean getAlwaysAimTurret() {
        return alwaysAimTurret;
    }

    public boolean getAlwaysSetVelocity() {
        return alwaysSetVelocity;
    }

    public boolean getAlwaysAimPitch() {
        return alwaysAimPitch;
    }

    public PidfController getShooterController() {
        return shooterController;
    }

    public PidfController getTurretController() {
        return turretController;
    }

    public DcMotorEx getShooterMotor() {
        return topShooterMotor;
    }

    public CRServo getTurretServo() {
        return primaryTurretServo;
    }

    public Servo getPitchServo() {
        return pitchServo;
    }

    public Double getVelocityTarget() {
        return velocityTarget;
    }

    public double getPitchTarget(){
        return pitchTarget;
    }

    public double getTurretTarget(){
        return turretTarget;
    }

    public double getRawPitchPos() {
        return (pitchEncoder.getVoltage()/pitchEncoder.getMaxVoltage());
    }

    public double getTurretRawPose() {
        return turretEncoder.getVoltage()/turretEncoder.getMaxVoltage();
    }

    public Double getVelocityRPM() {
        return (Math.abs(topShooterMotor.getVelocity()) * 60)/Params.SHOOTER_TICKS_PER_REV;
    }

    public double getPitchDegrees() {
        double rawPitchPos = (pitchEncoder.getVoltage() / pitchEncoder.getMaxVoltage());
        return rawPitchToDegrees(rawPitchPos);
    }

    public double getTurretDegrees() {
        double rawTurretPos = getTurretRawPose() + 0.987 * crossovers;
        double encoderOffset = (rawTurretPos - Params.TURRET_ENCODER_ZERO_OFFSET);
        double unconverted = (encoderOffset * Params.TURRET_DEGREES_PER_REV) + Params.TURRET_POSITION_OFFSET;
        return modularConversion(unconverted);
    }

    public int getCrossovers() {
        return crossovers;
    }

    public double rawPitchToDegrees(double rawPitchPos) {
        double encoderOffset = (rawPitchPos - Shooter.Params.PITCH_ENCODER_ZERO_OFFSET);
        return (encoderOffset * Params.PITCH_DEGREES_PER_REV) + Shooter.Params.PITCH_POSITION_OFFSET;
    }

    public double degreesToRawPitch(double degrees) {
        //TODO: Why did you remove this safety. double clippedDegrees = Range.clip(degrees, Params.MIN_PITCH_DEGREES, Params.MAX_PITCH_DEGREES);
        double pitchZeroOffset = (degrees - Params.PITCH_POSITION_OFFSET) / (Params.PITCH_DEGREES_PER_REV);
        return pitchZeroOffset + Params.PITCH_ENCODER_ZERO_OFFSET;
    }

    public void setVelocityTarget(Double velocityTargetRPM) {
        this.velocityTarget = velocityTargetRPM;
    }

    public void setPitchDegrees(Double targetPitchDegrees) {
        pitchTarget = degreesToRawPitch(targetPitchDegrees);
    }

    public void setTurretDegrees(Double targetDegrees) {
        if (targetDegrees == null){
            turretTarget = null;
            return;
        } else if (targetDegrees == 0){
            turretTarget = 0.0;
            return;
        }

        double turretTargetRaw = (targetDegrees - Math.toDegrees(robot.follower.getHeading())) * -1;
        double turretTargetModulo = robot.shooter.modularConversion(turretTargetRaw);
        turretTarget = Range.clip(turretTargetModulo, -160, 160);
        //TODO: Why are you only clipping to 160 and not 180? This seems like a band-aid.
    }

    public void setAlwaysAimTurret(boolean condition) {
        alwaysAimTurret = condition;
    }

    public void setAlwaysSetVelocity(boolean condition) {
        alwaysSetVelocity = condition;
    }

    public void setAlwaysAimPitch(boolean condition) {
        alwaysAimPitch = condition;
    }

    public double modularConversion(double n) {
        return (Math.floorMod((long)(n + 180), 360) - 180);
    }

    public void countCrossovers() {

        prevVoltage = currentVoltage;
        currentVoltage = getTurretRawPose();

        if (Math.abs(currentVoltage - prevVoltage) > Params.CROSSOVER_THRESHOLD) {
            if (currentVoltage > prevVoltage) {
                crossovers--;
            }
            else if (currentVoltage < prevVoltage) {
                crossovers++;
            }
        }
    }

    public double velDropCompensationWithPitch() {
        //Note: This would be added to Degrees before being converted to raw pitch
        return getShooterController().getError() * Params.VEL_DROP_PITCH_SCALE;
    }

    public void stopShooterMotor() {
        if (velocityTarget != null) {
            velocityTarget = null;
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

    public void stopShooter() {
        stopShooterMotor();
        stopPitchServo();
        stopTurret();
    }

    public boolean isFlywheelOnTarget(double tolerance) {
        boolean isOnTarget = false;
        if (velocityTarget != null) {
            isOnTarget = (Math.abs(velocityTarget - getVelocityRPM()) <= tolerance);
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

    public boolean isShooterReady(double flywheelTolerance, double turretTolerance, double pitchTolerance) {
        return (velocityTarget == null || isFlywheelOnTarget(flywheelTolerance))
                && (turretTarget == null || isTurretOnTarget(turretTolerance))
                && (pitchTarget == null || isPitchOnTarget(pitchTolerance));
    }

    public boolean isShooterReady() {
        return isShooterReady(Params.SHOOTER_TOLERANCE_RPM, Params.TURRET_TOLERANCE, Params.PITCH_TOLERANCE);
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
        }
    }

    public void turretTask() {
        double output = 0;
        countCrossovers();

        if (turretTarget != null) {
            double currentPosition = getTurretDegrees();
            output = turretController.calculate(turretTarget, currentPosition) + 0.0004;
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
        //Find rpm and pitch from the shootparams table based on distance to goal.
        ShootParams.Entry shootParams = Shooter.shootParamsTable.get(aimInfo.getDistanceToGoal());

        //TODO: All this logic could be written a lot better but for now I dont want to confuse yall?
        if(compensateForVelDropWithPitch) {
            double pitchCompensation = velDropCompensationWithPitch();
            if(alwaysAimPitch) {
                setPitchDegrees(shootParams.region.tiltAngle + pitchCompensation);
            }
            else {
                setPitchDegrees(pitchTarget + pitchCompensation);
            }
        }
        else {
            if(alwaysAimPitch) {
                setPitchDegrees(shootParams.region.tiltAngle);
            }
            else {
                setPitchDegrees(pitchTarget);
            }
        }

        //Aim turret, pitch, and velocity based on the boolean settings using the shootparams table.
        if (alwaysAimTurret) {
            setTurretDegrees(aimInfo.getAngleToGoal());
        } else {
            setTurretDegrees(turretTarget);
        }

        if (alwaysSetVelocity) {
            setVelocityTarget(shootParams.outputs[0]);
        }
        //Always setting pitch to the calculated angle from the shootparams table.
        setVelocityTarget(velocityTarget);
    }
}
