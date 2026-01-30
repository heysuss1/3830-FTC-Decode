package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.88)
            .forwardZeroPowerAcceleration(-31.0)
            .lateralZeroPowerAcceleration(-74.294)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.0001, 0.000005, 0.6, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(2.8, 0, 0.06, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.005,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0003, 0.6, 0.0001))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0.0001, 0.000005, 0.6, 0.01))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.13, 0, 0.01, 0.02))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(false)
            .holdPointTranslationalScaling(.45) //maybe this will work?
            .useSecondaryTranslationalPIDF(true);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.284)
            .yVelocity(57.58)
            .useBrakeModeInTeleOp(true);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.2859)
            .strafePodX(-4.8607)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
