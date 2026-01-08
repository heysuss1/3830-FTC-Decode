package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.TestHardware;
import org.firstinspires.ftc.teamcode.TestShooter;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;



@Config
@TeleOp (name="Spin Both Servos")
public class TestBothServos extends LinearOpMode {

    CRServo primaryTurretServo;
    CRServo secondaryTurretServo;
    AnalogInput turretEncoder;
    public static double gearRatio = .535;

    double zeroOffset;


//    double currentDegrees = (encoderOffset * TestShooter.Params.TURRET_DEGREES_PER_REV) + TestShooter.Params.TURRET_POSITION_OFFSET;

    public double getTurretDegrees(double rawTurretPos) {
        double encoderOffset = (rawTurretPos - zeroOffset);
        double currentDegrees = (encoderOffset * (gearRatio * 360)) + TestShooter.Params.TURRET_POSITION_OFFSET;
        return currentDegrees; //+ crossovers * Shooter.Params.TURRET_DEGREES_PER_REV;


        //TODO: test in the TurretTester class
    }




    public void runOpMode(){
        primaryTurretServo = hardwareMap.get(CRServo.class, "primaryTurretServo");
        secondaryTurretServo = hardwareMap.get(CRServo.class, "secondaryTurretServo");
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        waitForStart();

        zeroOffset = (turretEncoder.getVoltage()/3.3);

        while(opModeIsActive()){


            if (gamepad1.square){
                primaryTurretServo.setPower(1);
                secondaryTurretServo.setPower(primaryTurretServo.getPower());
            } else if(gamepad1.circle){
                primaryTurretServo.setPower(-1);
                secondaryTurretServo.setPower(primaryTurretServo.getPower());
            }else {
                primaryTurretServo.setPower(0);
                secondaryTurretServo.setPower(primaryTurretServo.getPower());
            }
            telemetry.addData("Raw Turret Position", (turretEncoder.getVoltage()/ turretEncoder.getMaxVoltage() - zeroOffset));
            telemetry.addData("Turret In Degrees", getTurretDegrees(turretEncoder.getVoltage()/ turretEncoder.getMaxVoltage()));
            telemetry.addData("Zero Offset", zeroOffset);
            telemetry.update();
        }
    }
}
