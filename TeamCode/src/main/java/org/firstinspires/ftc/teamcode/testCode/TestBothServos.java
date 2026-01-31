package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp (name="Spin Both Servos")
public class TestBothServos extends LinearOpMode {

   Robot robot;
    public static double gearRatio = .535;

    double zeroOffset;




    public void runOpMode(){
        robot = new Robot(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){


//            Shooter.Params.TURRET_DEGREES_PER_REV = 360 * gearRatio;
            if (gamepad1.square){
                robot.shooter.getPrimaryTurretServo().setPower(0.5);
            } else if(gamepad1.circle){
                robot.shooter.getPrimaryTurretServo().setPower(-0.5);

            } else if (gamepad1.triangle){
                robot.shooter.getSecondaryTurretServo().setPower(0.5);
            } else if (gamepad1.cross){
                robot.shooter.getSecondaryTurretServo().setPower(-0.5);

            }else {
                robot.shooter.getPrimaryTurretServo().setPower(0);
                robot.shooter.getSecondaryTurretServo().setPower(0);


            }

            robot.shooter.turretTask();
            telemetry.addData("Raw Turret Position", robot.shooter.getTurretRawPose());
            telemetry.addData("Turret In Degrees", robot.shooter.getTurretDegrees());
            telemetry.addData("Zero Offset", zeroOffset);
            telemetry.addData("Crossovers", robot.shooter.getCrossovers());
            telemetry.update();
        }
    }
}
