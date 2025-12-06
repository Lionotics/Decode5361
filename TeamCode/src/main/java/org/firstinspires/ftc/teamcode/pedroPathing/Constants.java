package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * PedroPathing constants using drive motor encoders for localization.
 *
 * IMPORTANT:
 * - Motor names here MUST match your configuration (and your DriveTrain.java).
 *   From your code they look like: "frontLeft", "frontRight", "backLeft", "backRight".
 * - All the numbers here (mass, constraints, tick multipliers, PIDF) are placeholders
 *   you should replace after running Pedro's tuners.
 */
public class Constants {

    // --- Follower constants: mass, PIDF, etc. ---
    public static FollowerConstants followerConstants = new FollowerConstants()
            // robot mass in kg (rough guess – put your real value here)
            .mass(13.7)
            // You will tune these in Panels (Pedro tuning OpModes) and copy back here.
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.1, 0.0, 0.01, 0.0
            ))
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03, 0.0, 0.0, 0.0
            ))
            .drivePIDFCoefficients( new FilteredPIDFCoefficients(  0.03, 0.0, 0.0, 0.0,0) );


    // --- Drivetrain constants: motor names + directions for mecanum ---
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)

            // These names must match your hardwareMap + DriveTrain.java
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")

            // Directions should match whatever you do in DriveTrain.java
            // In your DriveTrain you reversed the left side, so:
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // --- Drive Encoder localizer (no dead wheels) ---
    // Uses the encoders of the four drive motors for localization.
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")

            // Encoder directions – usually FORWARD to start; flip if tests look wrong.
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)

            // Robot size in inches (rough guess, change to your real robot)
            .robotWidth(15)
            .robotLength(13)

            // These three come from the Pedro localization tuners
            // (Forward, Strafe, Turn) – placeholders until you tune.
            .forwardTicksToInches(1.0)
            .strafeTicksToInches(1.0)
            .turnTicksToInches(1.0);

    // --- Path constraints for autonomous paths ---
    // maxTranslationalVel, maxTranslationalAccel, maxAngularVel, maxAngularAccel
    public static PathConstraints pathConstraints =
            new PathConstraints(40.0, 40.0,
                    Math.toRadians(180), Math.toRadians(180));

    /**
     * Pedro 2.0 style factory for the follower.
     * NextFTC will use this in your PedroOpMode.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)  // <--- NO ODOM WHEELS NEEDED
                .build();
    }
}
