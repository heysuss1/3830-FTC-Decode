

package org.firstinspires.ftc.teamcode.controllers;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.TICKS_PER_REVOLUTION;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VelocityController {
    public double kP = 0.005; double kI = 0; double kD = 0;
    public double kS = 1.26; //calculated by turning the flywheel
    public double kV = 0.0024; //calculated by doing bestz line for power.
    public HardwareMap hwMap;
    public double batteryVoltage;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);
    PIDController pid = new PIDController(kP, kI, kD);
    PidfController pidf = new PidfController(0.0025, 0, 0, 0.000246, 100);
    public double RPMtoTPS(int rpm) {
        return (rpm * TICKS_PER_REVOLUTION / 60.0);
    }
    public VelocityController(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public double getBatteryVoltage(){
        return hwMap.voltageSensor.iterator().next().getVoltage();
    }

    public void setCoefficients(double kP, double kI, double kD){
        pid.setPID(kP, kI, kD);
    }

    public double getPower(double currentRPM, int targetRPM){
        return pidf.calculate(targetRPM, currentRPM);
    }
}

