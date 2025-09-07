package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class CoolTeleOP extends LinearOpMode {
    Intake intake;
    public void runOpMode() {
        intake = new Intake();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad2.a){
                intake.initialize();
            }
        }
    }
}
