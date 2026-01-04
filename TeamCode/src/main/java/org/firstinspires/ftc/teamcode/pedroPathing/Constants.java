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

/**
 * PedroPathing + NextFTC constants.
 * Uses mecanum drivetrain and a goBILDA Pinpoint odometry pod named "Odometry".
 */
public class Constants {

    // ---------------- Follower (mass + PIDF etc.) ----------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            // rough mass in kg – replace with real robot mass
            .mass(12.7)

            // You’ll eventually tune these with Pedro’s tuners;
            // these values are just placeholders.
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03, 0.0, 0.0, 0.0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.1, 0.0, 0.01, 0.0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.03, 0.0, 0.0, 0.0, 0.0
            ));

    // ---------------- Mecanum drivetrain constants ----------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            // THIS is where maxPower belongs in Pedro 2.0
            .maxPower(1.0)

            // Motor names must match your RC config and DriveTrain.java
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")

            // Directions must match how the robot actually drives
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            // Placeholders until you run the Forward/Lateral Velocity tuners
            .xVelocity(40.0)   // in/s, forward max velocity (tune later)
            .yVelocity(40.0);  // in/s, strafe max velocity (tune later)

    // ---------------- Pinpoint odometry localizer ----------------
    // goBILDA Pinpoint I2C device named "Odometry"
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Pod offsets RELATIVE TO ROBOT CENTER (in inches).
            // Measure these and replace 0.0s later:
            .forwardPodY(6.0)      // + = pod is in front of center, - = behind
            .strafePodX(0.0)       // + = pod is to the left,  - = to the right

            // Units + hardware map name + encoder model
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

            // Encoder directions (flip if your X/Y go the wrong way)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ---------------- Path constraints (end-of-path tolerances) ----------------
    // Constructor signature in Pedro 2.0: (tValue, velocity, translational, heading)
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // tValue: how far along the curve (0–1) before it can end
            100.0,  // velocity constraint (in/s) for "done"
            1.0,    // translational error allowed (in)
            1.0     // heading error allowed (rad)
    );

    // ---------------- Factory: create the Follower ----------------
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
