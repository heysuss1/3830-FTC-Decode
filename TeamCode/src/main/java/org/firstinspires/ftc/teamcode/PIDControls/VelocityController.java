

package org.firstinspires.ftc.teamcode.PIDControls;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.TICKS_PER_REVOLUTION;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VelocityController {
    public double kP = 0; double kI = 0; double kD = 0.00001;
    public double kS = 0.1279; //calculated by turning the flywheel
    public double kV = 0.000388; //calculated by doing bestz line for power.
    public HardwareMap hwMap;
    public double batteryVoltage;
    public final double referenceVoltage = 12.9;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

    static PIDFController pidf =  new PIDFController(0.00001, 0.0, 0.0, 0.00024, 0.0);
    PIDController pid = new PIDController(kP, kI, kD);
    public double RPMtoTPS(int rpm) {
        return (rpm * TICKS_PER_REVOLUTION / 60.0);
    }
    public VelocityController(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public double getBatteryVoltage(){
        batteryVoltage = hwMap.voltageSensor.iterator().next().getVoltage();
        return batteryVoltage;
    }
    public static double getPower(double currentVel, int targetVel){
        return pidf.calculate(targetVel, currentVel);
    }
    public double getPower(double currentVel, int targetVel, double currentBattery){
        batteryVoltage = targetVel = (int)RPMtoTPS(targetVel);
        double ff = feedforward.calculate(targetVel) / getBatteryVoltage();
        return pid.calculate(currentVel, targetVel);
    }

}

