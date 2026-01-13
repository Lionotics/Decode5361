package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;


@Config
public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() {
    }
    private MotorEx frontLeft, frontRight, backLeft, backRight;

    public GoBildaPinpointDriver odometry;

    private MotorEx[] motors;
    private IMU imu;




    // --- Tag world estimate (odometry frame) ---
    public static boolean haveBlueTagEstimate = false;
    public static double blueTagX_in = 0.0;
    public static double blueTagY_in = 0.0;

    // If your camera is not at robot center, put offsets here (inches).
// +X = forward, +Y = left (matches what you described).
    public static double camOffsetX_in = 0.0;
    public static double camOffsetY_in = 0.0;

    // How aggressively we trust new measurements (0..1). Higher = updates faster.
    public static double tagEstimateAlpha = 0.25;

    // Turning behavior when tag is NOT visible
    public static double kP_noTag = 0.05;     // power per degree
    public static double maxTurn_noTag = 0.6;
    public static double minTurn_noTag = 0.08;

    // Turning behavior when tag IS visible (fine alignment)
    public static double kP_inTag = 0.05;
    public static double maxTurn_inTag = 0.25;
    public static double minTurn_inTag = 0.06;

    public  static double searchPowerBeforeTag = 0.35;


    private static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }

    private static double signedMinPower(double power, double minAbs) {
        if (Math.abs(power) < minAbs) return Math.copySign(minAbs, power);
        return power;
    }

    private void updateBlueTagEstimate(AprilTagDetection d) {
        if (d == null || d.ftcPose == null || odometry == null) return;

        Pose2D pose = odometry.getPosition();

        double rx = pose.getX(DistanceUnit.INCH);
        double ry = pose.getY(DistanceUnit.INCH);
        double hDeg = pose.getHeading(AngleUnit.DEGREES);

        // Camera position in world frame (approx)
        double hRad = Math.toRadians(hDeg);
        double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
        double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

        // Direction from camera to tag in world frame
        double dirDeg = hDeg + d.ftcPose.bearing;   // bearing positive = camera must turn left/right :contentReference[oaicite:5]{index=5}
        double dirRad = Math.toRadians(dirDeg);

        double measTagX = camX + d.ftcPose.range * Math.cos(dirRad);
        double measTagY = camY + d.ftcPose.range * Math.sin(dirRad);

        if (!haveBlueTagEstimate) {
            blueTagX_in = measTagX;
            blueTagY_in = measTagY;
            haveBlueTagEstimate = true;
        } else {
            // EMA filter to smooth noise
            blueTagX_in = (1 - tagEstimateAlpha) * blueTagX_in + tagEstimateAlpha * measTagX;
            blueTagY_in = (1 - tagEstimateAlpha) * blueTagY_in + tagEstimateAlpha * measTagY;
        }
    }







    public void initialize() {
      //  frontLeft = new MotorEx("frontLeft");
       // frontRight = new MotorEx("frontRight");
       // backLeft = new MotorEx("backLeft");
       // backRight = new MotorEx("backRight");

        frontLeft = new MotorEx("backLeft");
        frontRight = new MotorEx("backRight");
        backLeft = new MotorEx("frontLeft");
        backRight = new MotorEx("frontRight");


        odometry = OpModeData.INSTANCE.getHardwareMap().get(GoBildaPinpointDriver.class, "Odometry");



        frontLeft.reverse();
        backLeft.reverse();

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        initIMU(OpModeData.INSTANCE.getHardwareMap());
    }

    public void initIMU(HardwareMap hwMap) {
        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odometry.resetPosAndIMU();
    }

    public Command Drive(GamepadEx gamepad, boolean robotOreinted) {
        return new MecanumDriverControlled(motors, gamepad, robotOreinted, imu);
    }

    public void setTurnPower(double turn) {
        turn = Range.clip(turn, -1.0, 1.0);

        // Turn in place: left side forward, right side backward (signs may be flipped if you prefer)
        frontLeft.setPower(turn);
        backLeft.setPower(turn);
        frontRight.setPower(-turn);
        backRight.setPower(-turn);
    }

    public void stopDrive() {
        setTurnPower(0.0);
    }





    final int BLUE_GOAL_TAG_ID = 20;

    // Tune these:

    public static double desiredTilt = 0;
    public static double deadbandDeg = 2.0;
    public static long timeoutMs = 25000;

    public  static double turnPowerValue = 0.33;

    // tiny “mutable holders” for lambdas
    final long[] startTime = new long[1];
    final boolean[] sawTag = new boolean[1];
    final double[] lastErrorDeg = new double[1];


    public Command faceBlueGoal = new LambdaCommand()
            .setStart(() -> {
                startTime[0] = System.currentTimeMillis();
                sawTag[0] = false;
                lastErrorDeg[0] = 999;
            })
            .setUpdate(() -> {
                AprilTagDetection d = Webcam.INSTANCE.getDetectionById(BLUE_GOAL_TAG_ID);

                // Always keep odometry-based estimate fresh when tag is visible
                if (d != null && d.ftcPose != null) {
                    updateBlueTagEstimate(d);
                }

                // If tag is visible: fine align using BEARING
                if (d != null && d.ftcPose != null) {
                    sawTag[0] = true;

                    double errorDeg = wrapDeg(desiredTilt - d.ftcPose.bearing);
                    lastErrorDeg[0] = errorDeg;

                    if (Math.abs(errorDeg) < deadbandDeg) {
                        DriveTrain.INSTANCE.setTurnPower(0.0);
                        return;
                    }

                    double turn = Range.clip(kP_inTag * errorDeg, -maxTurn_inTag, maxTurn_inTag);
                    turn = signedMinPower(turn, minTurn_inTag);
                    DriveTrain.INSTANCE.setTurnPower(turn);
                    return;
                }

                // Tag NOT visible: use odometry to "pre-aim" toward where we think the tag is
                if (haveBlueTagEstimate && odometry != null) {
                    Pose2D pose = odometry.getPosition();
                    double rx = pose.getX(DistanceUnit.INCH);
                    double ry = pose.getY(DistanceUnit.INCH);
                    double hDeg = pose.getHeading(AngleUnit.DEGREES);

                    double dx = blueTagX_in - rx;
                    double dy = blueTagY_in - ry;

                    // heading to face the tag (in your x-forward, y-left frame)
                    double dirToTagDeg = Math.toDegrees(Math.atan2(dy, dx));

                    // We want camera bearing to be desiredTilt, so:
                    // bearing ≈ dirToTag - heading  =>  headingTarget ≈ dirToTag - desiredTilt
                    double headingTargetDeg = dirToTagDeg - desiredTilt;

                    double errorDeg = wrapDeg(headingTargetDeg - hDeg);
                    errorDeg = -1 * errorDeg;
                    lastErrorDeg[0] = errorDeg;

                    if (Math.abs(errorDeg) < deadbandDeg) {
                        DriveTrain.INSTANCE.setTurnPower(0.0);
                        return;
                    }

                    double turn = Range.clip(kP_noTag * errorDeg, -maxTurn_noTag, maxTurn_noTag);
                    turn = signedMinPower(turn, minTurn_noTag);
                    DriveTrain.INSTANCE.setTurnPower(turn);
                    return;
                }

                // If we have NO estimate yet: do a gentle alternating search (prevents "always clockwise")
                DriveTrain.INSTANCE.setTurnPower(searchPowerBeforeTag);
            })
            .setIsDone(() -> {
                long elapsed = System.currentTimeMillis() - startTime[0];
                if (elapsed > timeoutMs) {
                    return true;
                }
                if (!sawTag[0]) {
                    return false;
                } // keep hunting until timeout
                return Math.abs( lastErrorDeg[0])  < deadbandDeg;
            })
            .setStop(interrupted -> {
                DriveTrain.INSTANCE.stopDrive();
                // resume normal teleop drive when done (or interrupted)
            });

}


