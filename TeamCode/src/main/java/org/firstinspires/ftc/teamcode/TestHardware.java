package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class TestHardware {
    public TestShooter shooter;

    public Follower follower;

    public TestHardware(HardwareMap hardwareMap, Telemetry telemetry){
        this.follower = (Constants.createFollower(hardwareMap));
        shooter = new TestShooter(hardwareMap, telemetry, follower);
    }
}
