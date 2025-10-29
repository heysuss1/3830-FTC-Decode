package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain {

    //Initialize motors
    public DcMotorEx lf;
    public DcMotorEx rf;
    public Telemetry telemetry;
    public DcMotorEx lb;
    public DcMotorEx rb;
    private double speed;

    //Get motors from the hardware map
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        lf = hwMap.get(DcMotorEx.class, "lf");
        lf.setPower(0);
        rf = hwMap.get(DcMotorEx.class, "rf");
        rf.setPower(0);
        lb = hwMap.get(DcMotorEx.class, "lb");
        lb.setPower(0);
        rb = hwMap.get(DcMotorEx.class, "rb");
        rb.setPower(0);
    }

    public void setBrakeMode(){
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    //Get the current max speed of the robot.
    public double getSpeed() {
        return speed;
    }
    //Set the current max speed of the robot, has to be between -1 and 1.
    public void setSpeed(double speed){
        this.speed = Range.clip(speed, -1, 1);
    }


    //Do math to move the robot!!!
    public void moveRobot(Gamepad gamepad1){
        double forward;
        double sideways;
        double turning;
        double max;
        double scaleFactor;

        forward = -(Math.atan(5 * gamepad1.left_stick_y) / Math.atan(5));
        sideways = (Math.atan(5 * gamepad1.left_stick_x) / Math.atan(5));
        turning = (Math.atan(5 * gamepad1.right_stick_x) / Math.atan(5));
        max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
        if (max > speed) {
            scaleFactor = speed/max;
        } else {
            scaleFactor = speed;
        }
        scaleFactor *= Math.max(Math.abs(1), 0.2);

        setPower((forward - sideways - turning)*scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
    }

    //Set power to all of the motors at once.
    public void setPower(double fr, double br, double fl, double bl){
        rf.setPower(Range.clip(fr, -speed, speed));
        rb.setPower(Range.clip(br, -speed, speed));
        lf.setPower(Range.clip(fl, -speed, speed));
        lb.setPower(Range.clip(bl, -speed, speed));
    }
}
