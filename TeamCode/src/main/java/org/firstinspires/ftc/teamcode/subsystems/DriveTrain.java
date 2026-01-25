package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {

    //TODO: Add field centric control

    private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
    private Telemetry telemetry;
    double maxSpeed = 1.0;
    boolean slowTurning = true;

    public DriveTrain(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setSlowTurning(boolean turnSlow) {
        slowTurning = turnSlow;
    }
    public void setMaxSpeed(double speed) {
        maxSpeed = Range.clip(speed, 0.0, 1.0);
    }

    private void setPower(double rightFrontPower, double leftFrontPower, double rightRearPower, double leftRearPower) {
        rightFront.setPower(Range.clip(leftRearPower, -maxSpeed, maxSpeed));
        leftFront.setPower(Range.clip(rightFrontPower, -maxSpeed, maxSpeed));
        rightRear.setPower(Range.clip(leftFrontPower, -maxSpeed, maxSpeed));
        leftRear.setPower(Range.clip(rightRearPower, -maxSpeed, maxSpeed));
    }

    public void driveTask(Gamepad gamepad1)
    {
        double y = -(Math.atan(5 * gamepad1.left_stick_y) / Math.atan(5));
        double x = (Math.atan(5 * gamepad1.left_stick_x) / Math.atan(5)) * 1.1; // Strafing compensation
        double turning = (Math.atan(5 * gamepad1.right_stick_x) / Math.atan(5));

        if (slowTurning) {
            turning *= 0.5; //Slow down turning
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turning), 1);
        double frontLeftPower = (y + x + turning) / denominator;
        double backLeftPower = (y - x + turning) / denominator;
        double frontRightPower = (y - x - turning) / denominator;
        double backRightPower = (y + x - turning) / denominator;

        setPower(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
    }
}

