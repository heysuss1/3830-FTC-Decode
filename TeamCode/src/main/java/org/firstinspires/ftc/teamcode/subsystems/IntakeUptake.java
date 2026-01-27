package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot;

public class IntakeUptake {


    public LED redLED;
    public LED greenLED;

    public enum intakeUptakeStates {
        OFF,
        INTAKING,
        UPTAKING,
        OUTTAKING
    }
    public static class Params {
        public static final double HAS_BALL_1_DISTANCE_THRESHOLD = 2.45; //inches
        public static final double HAS_BALL_2_DISTANCE_THRESHOLD = 1.4;
        public static final double HAS_BALL_3_DISTANCE_THRESHOLD = 1.9;

        public static final double BLOCKING_SERVO_1_OPEN_POS = 0.6;
        public static final double BLOCKING_SERVO_1_CLOSE_POS = .82;

        public static final double BLOCKING_SERVO_2_OPEN_POS = 0.37;
        public static final double BLOCKING_SERVO_2_CLOSE_POS = .2;
    }

    private final Telemetry telemetry;

    private final Servo blockingServo1, blockingServo2;
    private final DcMotorEx intakeMotor, uptakeMotor;
    private final RevColorSensorV3 colorSensor1, colorSensor2, colorSensor3;

    private intakeUptakeStates intakeUptakeState = intakeUptakeStates.OFF;

    public IntakeUptake(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        redLED = hwMap.get(LED.class, "red");
        greenLED = hwMap.get(LED.class, "green");

        //Lower port number is green, higher port number is red

        blockingServo1 = hwMap.get(Servo.class, "blockingServo1");
        blockingServo2 = hwMap.get(Servo.class, "blockingServo2");

        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        uptakeMotor = hwMap.get(DcMotorEx.class, "uptakeMotor");
        uptakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        uptakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uptakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeMotor.setPower(0);

        colorSensor1 = hwMap.get(RevColorSensorV3.class, "colorSensor1");
        colorSensor2 = hwMap.get(RevColorSensorV3.class, "colorSensor2");
        colorSensor3 = hwMap.get(RevColorSensorV3.class, "colorSensor3");

    }

    public boolean isUptakeEmpty() {
        return  (colorSensor1.getDistance(DistanceUnit.INCH) > Params.HAS_BALL_1_DISTANCE_THRESHOLD) &&
                (colorSensor2.getDistance(DistanceUnit.INCH) > Params.HAS_BALL_2_DISTANCE_THRESHOLD) &&
                (colorSensor3.getDistance(DistanceUnit.INCH) > Params.HAS_BALL_3_DISTANCE_THRESHOLD);
    }

    public Servo getBlockingServo1() {
        return blockingServo1;
    }

    public Servo getBlockingServo2() {
        return blockingServo2;
    }

    public boolean hasBall1(){
        return colorSensor1.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_1_DISTANCE_THRESHOLD;
    }
    public boolean hasBall2(){
        return colorSensor2.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_2_DISTANCE_THRESHOLD;
    }
    public boolean hasBall3(){
        return colorSensor2.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_3_DISTANCE_THRESHOLD;
    }

    public double getColorSensor1Distance(){
        return colorSensor1.getDistance(DistanceUnit.INCH);
    }
    public double getColorSensor2Distance(){
        return colorSensor2.getDistance(DistanceUnit.INCH);
    }
    public double getColorSensor3Distance(){
        return colorSensor3.getDistance(DistanceUnit.INCH);
    }


    public double getNumberOfBallsStored() {
        boolean hasBall1 = (colorSensor1.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_1_DISTANCE_THRESHOLD);
        boolean hasBall2 = (colorSensor2.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_2_DISTANCE_THRESHOLD);
        boolean hasBall3 = (colorSensor3.getDistance(DistanceUnit.INCH) < Params.HAS_BALL_3_DISTANCE_THRESHOLD);

        if (hasBall1 && hasBall3){
            return 3;
        } else if (hasBall2){
            return 2 + (hasBall1? 1: 0);
        }
        return  (hasBall3 ? 1 : 0) +
                ((hasBall3 && hasBall2) ? 1 : 0) +
                ((hasBall3 && hasBall2 && hasBall1) ? 1 : 0);
    }

    public void setIntakeUptakeMotorPower(double intakePower, double uptakePower) {
        intakeMotor.setPower(intakePower);
        uptakeMotor.setPower(uptakePower);
    }

    public void setIntakeUptakeMode(intakeUptakeStates pass) {
        intakeUptakeState = pass;
    }

    public void openBlockingServo() {
        blockingServo1.setPosition(Params.BLOCKING_SERVO_1_OPEN_POS);
        blockingServo2.setPosition(Params.BLOCKING_SERVO_2_OPEN_POS);
    }

    public void closeBlockingServo() {
        blockingServo1.setPosition(Params.BLOCKING_SERVO_1_CLOSE_POS);
        blockingServo2.setPosition(Params.BLOCKING_SERVO_2_CLOSE_POS);
    }


    public void toggleGreenRedLight(){
        if (getNumberOfBallsStored() == 3){
            greenLED.on();
            redLED.off();
        } else if (getNumberOfBallsStored() == 2){
            greenLED.on();
            redLED.on();
        } else if (getNumberOfBallsStored() == 1){
            redLED.on();
            greenLED.off();
        } else {
            redLED.off();
            greenLED.off();
        }
    }


    public void intakeUptakeTask() {
        switch (intakeUptakeState) {
            case OFF:
                setIntakeUptakeMotorPower(0,0);
                break;
            case INTAKING:
                setIntakeUptakeMotorPower(.75, 1);  // <----- change these values to the correct ones!!111!11!!!!
                break;
            case UPTAKING:
                setIntakeUptakeMotorPower(1, 1);
                break;
            case OUTTAKING:
                setIntakeUptakeMotorPower(-1, -1);
                break;
        }
        toggleGreenRedLight();
        
        if (!Robot.inComp) {
            telemetry.addLine("\nIntake/Uptake Info:");
            telemetry.addData("Intake Power",intakeMotor.getPower());
            telemetry.addData("Uptake Power", uptakeMotor.getPower());
            telemetry.addData("Color Sensors Distances", "Ball 1 Sensor: %f, Ball 2 Sensor: %f, Ball 3 Sensor: %f",
                    colorSensor1.getDistance(DistanceUnit.INCH),
                    colorSensor2.getDistance(DistanceUnit.INCH),
                    colorSensor3.getDistance(DistanceUnit.INCH));
            telemetry.addData("Number of Balls Stored: ", getNumberOfBallsStored() + ", Transfer Empty: " + isUptakeEmpty());
            telemetry.update();
        }
    }
}
