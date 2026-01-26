package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-36.027911422538544)
            .lateralZeroPowerAcceleration(-67.6582427832326)
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.4,0,0.05,0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.1,0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6,0,0.02,0))
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.00001,0.6,0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.000005,0.6,0.01))
            .mass(19.5);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.3, 1.2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.5)
            .strafePodX(-6.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(78.32932455708664)
            .yVelocity(62.7116675189161)
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .useBrakeModeInTeleOp(true);
}
