package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TestHardware {
    public TestShooter shooter;

    public Follower follower;
    public DcMotorEx intakeMotor1, uptakeMotor;


    public TestHardware(HardwareMap hardwareMap, Telemetry telemetry){
        this.follower = (Constants.createFollower(hardwareMap));
        shooter = new TestShooter(hardwareMap, telemetry, follower);

        intakeMotor1 = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor1.setPower(0);

        uptakeMotor = hardwareMap.get(DcMotorEx.class, "uptakeMotor");
        uptakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        uptakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uptakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeMotor.setPower(0);
    }

    public void setIntakeUptakePower(double intakePower, double uptakePower){
        uptakeMotor.setPower(uptakePower);
        intakeMotor1.setPower(intakePower);
    }



}
