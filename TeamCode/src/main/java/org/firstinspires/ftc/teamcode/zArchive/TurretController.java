package org.firstinspires.ftc.teamcode.zArchive;

import com.arcrobotics.ftclib.controller.PIDController;

public class TurretController {

    static private double kP;
    static private double kI;
    static private double kD;


    static PIDController pid = new PIDController(kP, kI, kD);
    public static double getPower(int currentPosition, int targetPosition){
        return pid.calculate(currentPosition, targetPosition);
    }
}
