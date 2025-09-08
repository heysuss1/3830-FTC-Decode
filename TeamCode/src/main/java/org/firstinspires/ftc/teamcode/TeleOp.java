package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems.IntakeNFTC;

public class TeleOp extends LinearOpMode {
    Hardware robot;
    public void runOpMode() {

        waitForStart();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();
        while (opModeIsActive()){
        }
    }
}
