package org.firstinspires.ftc.teamcode.subsystems.NextFTCSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

public class Intake extends Subsystem {
    public static final Intake INSTANCE = new Intake();

    public DcMotorEx motor;



//    private Intake(){}

    public void initialize(){
        motor = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "intakeMotor");
    }
    LambdaCommand startIntake = new LambdaCommand()
            .setStart(() -> {
                // Runs on start
            motor.setPower(0.4);
            });
    LambdaCommand stopIntake = new LambdaCommand()
            .setStart(() -> {
                // Runs on start
                motor.setPower(0);
            });

}
