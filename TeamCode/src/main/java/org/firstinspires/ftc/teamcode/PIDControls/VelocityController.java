

package org.firstinspires.ftc.teamcode.PIDControls;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.TICKS_PER_REVOLUTION;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class VelocityController {
    public double kP = 0; double kI = 0; double kD = 0;
    public double kS = 1.26; //calculated by turning the flywheel
    public double kV = 0.0024; //calculated by doing bestz line for power.
    public HardwareMap hwMap;
    public double batteryVoltage;
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);
    PIDController pid = new PIDController(kP, kI, kD);
    public double RPMtoTPS(int rpm) {
        return (rpm * TICKS_PER_REVOLUTION / 60.0);
    }
    public VelocityController(HardwareMap hwMap){
        this.hwMap = hwMap;
    }

    public double getBatteryVoltage(){
        return hwMap.voltageSensor.iterator().next().getVoltage();
    }
    public double getPower(double currentVel, int targetVel, double voltage){
        double velError = targetVel - currentVel;
        double velFeedforward = feedforward.calculate(targetVel);
        double pidCorrection = pid.calculate( velError);

        double output =velFeedforward + pidCorrection;


        output = (output/voltage);

        return (Range.clip(output, -1, 1));

    }
}

