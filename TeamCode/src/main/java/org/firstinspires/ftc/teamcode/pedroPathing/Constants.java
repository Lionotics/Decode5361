package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Constants {
    public  static  double xVelocityRealValue = 69.9;
    public  static  double yVelocityRealValue = 47.59;



    // ---------------- Follower (mass + PIDF etc.) ----------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            // rough mass in kg – replace with real robot mass
             .mass(14.6)

            // You’ll eventually tune these with Pedro’s tuners;
            // these values are just placeholders.
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.3, 0.0, 0.01, 0.0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1, 0.1, 0.1, 0.1
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1, 0.0, 0.01, 0.0, 0.0
            ))
            .forwardZeroPowerAcceleration(-38.5)
            .lateralZeroPowerAcceleration(-86.65)
            ;

    // ---------------- Mecanum drivetrain constants ----------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            // THIS is where maxPower belongs in Pedro 2.0
            .maxPower(1.0)

            // Motor names must match your RC config and DriveTrain.java
        //    .leftFrontMotorName("frontLeft")
          //  .leftRearMotorName("backLeft")
          //  .rightFrontMotorName("frontRight")
          //  .rightRearMotorName("backRight")

            .leftFrontMotorName("backLeft")
            .leftRearMotorName("frontLeft")
            .rightFrontMotorName("backRight")
            .rightRearMotorName("frontRight")

            // Directions must match how the robot actually drives
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            // Placeholders until you run the Forward/Lateral Velocity tuners
            .xVelocity(xVelocityRealValue)   // in/s, forward max velocity (tune later)
            .yVelocity(yVelocityRealValue);  // in/s, strafe max velocity (tune later)

    // ---------------- Pinpoint odometry localizer ----------------
    // goBILDA Pinpoint I2C device named "Odometry"
    public static PinpointConstants localizerConstants = new PinpointConstants()
            // Pod offsets RELATIVE TO ROBOT CENTER (in inches).
            // Measure these and replace 0.0s later:
            .forwardPodY(-7.5)      // + = pod is in front of center, - = behind
            .strafePodX(-1.0)       // + = pod is to the left,  - = to the right

            // Units + hardware map name + encoder model
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("Odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)

            // Encoder directions (flip if your X/Y go the wrong way)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ---------------- Path constraints (end-of-path tolerances) ----------------
    // Constructor signature in Pedro 2.0: (tValue, velocity, translational, heading)
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,   // tValue: how far along the curve (0–1) before it can end
            100.0,  // velocity constraint (in/s) for "done"
            1.0,    // translational error allowed (in)
            Math.toRadians(2)     // heading error allowed (rad)
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
