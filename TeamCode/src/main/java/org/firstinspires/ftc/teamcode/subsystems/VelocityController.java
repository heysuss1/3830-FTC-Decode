package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.TICKS_PER_REVOLUTION;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

public class VelocityController {
    public double kP = 0; double kI = 0; double kD = 0.0000001;
    public double kS = 0.02; //calculated by turning the flywheel
    public double kV = 0.00041; //calculated by doing bestz line for power.
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);
    PIDController pid = new PIDController(kP, kI, kD);
    public double RPMtoTPS(int rpm) {
        return (rpm * TICKS_PER_REVOLUTION / 60.0);
    }
    public double getPower(double currentVel, int targetVel){
        targetVel = (int)RPMtoTPS(targetVel);
        return pid.calculate(currentVel, targetVel) + feedforward.calculate(targetVel);
    }
}
