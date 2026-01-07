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
            .mass(8)
            .forwardZeroPowerAcceleration(-36.154)
            .lateralZeroPowerAcceleration(-83.941)
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.003,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.007, 0, 0.00025, 0.6, 0.0001))
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("topShooterMotor")
            .rightRearMotorName("bottomShooterMotor")
            .leftRearMotorName("bottomShooterMotor")
            .leftFrontMotorName("topShooterMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(64.80)
            .yVelocity(44.957)
            .useBrakeModeInTeleOp(true);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.72)
            .strafePodX(-2.85)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
