package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.Set;


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
    public static boolean haveTagEstimate = false;
    public static double blueTagX_in = 0.0;
    public static double blueTagY_in = 0.0;

    // If your camera is not at robot center, put offsets here (inches).
// +X = forward, +Y = left (matches what you described).
    public static double camOffsetX_in = 0.0;
    public static double camOffsetY_in = 0.0;

    // How aggressively we trust new measurements (0..1). Higher = updates faster.
    public static double tagEstimateAlpha = 0.25;

    public  static  double jumpThreashold = 1.0;


    public static double lastFaceGoalAngleRelative = Double.NaN;
    public static double lastFaceGoalAngleAbsolute = Double.NaN;


    public  boolean followerIsActive = false;


    // If your robot turns the wrong direction, set this to -1 in dashboard.
    public static double faceGoalTurnSign = 1.0;

    public  static  boolean UsingOdemtryInsteadOfIMU = false; // if true, using odemtry, if false, using IMU



        // When TRUE: if the goal tag is NOT visible, do a simple IMU-based "turn left/right"
    // to the computed target heading (from the stored tag world estimate).
    // When FALSE: always run the current Pedro follower.turn() behavior.
    public static boolean useIMUFallbackWhenNoTag = true;

    // IMU fallback tuning
    public static double imuFallbackTurnPower = 0.35;     // 0..1
    public static double imuFallbackTimeoutSec = 10.75;    // safety timeout

    public static double faceGoalBearingDoneDeg = 2.0;  // finish when |bearing| < this


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


        double hDeg = 0;
        if (UsingOdemtryInsteadOfIMU) {
            hDeg = pose.getHeading(AngleUnit.DEGREES);
        } else {
            hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        // Camera position in world frame (approx)
        double hRad = Math.toRadians(hDeg);
        double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
        double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

        // Bearing is the left/right turn needed to point at tag center (deg)
        double dirDeg = hDeg + d.ftcPose.bearing;
        double dirRad = Math.toRadians(dirDeg);

        // IMPORTANT FIX:
        // range is point-to-point 3D distance; project onto the floor plane using elevation.


        double elevRad = Math.toRadians(d.ftcPose.elevation);

       // double horizontalRange = d.ftcPose.range * Math.cos(elevRad); // uncomment this code if not accounting for elevation causes problems

        double horizontalRange = d.ftcPose.range;



        double measTagX = camX + horizontalRange * Math.cos(dirRad);
        double measTagY = camY + horizontalRange * Math.sin(dirRad);



        // before blending:
        double dx = measTagX - blueTagX_in;
        double dy = measTagY - blueTagY_in;
        double jump = Math.hypot(dx, dy);

        // reject impossible jumps (tune 6-12 inches)
        if (haveTagEstimate && jump > jumpThreashold)  {
            return;
        }


        if (!haveTagEstimate) {
            blueTagX_in = measTagX;
            blueTagY_in = measTagY;
            haveTagEstimate = true;
        } else {
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

        odometry.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.RADIANS,0));

        blueTagX_in = 0;
        blueTagY_in = 0;
        haveTagEstimate  = false; // remember that I did this later. it might come back to bite me.

        frontLeft.reverse();
        backLeft.reverse();

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        initIMU(OpModeData.INSTANCE.getHardwareMap());


        Webcam.INSTANCE.setSoleTagID(GOAL_TAG_ID);

        followerIsActive = false;
    }

    public void initIMU(HardwareMap hwMap) {
        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        odometry.resetPosAndIMU();


        odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
        odometry.setOffsets(-1.0,-7.5,DistanceUnit.INCH);

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


    }

    public Command Drive(GamepadEx gamepad, boolean robotOriented) {
        // Please work
        // The normal NextFTC drive command
        MecanumDriverControlled inner =
                new MecanumDriverControlled(motors, gamepad, robotOriented, imu);

        // Wrap it so it REQUIRES (claims) this drivetrain subsystem
        return new Command() {

            @NotNull
            @Override
            public Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public void start() {
                inner.start();
            }

            @Override
            public void update() {
                inner.update();
            }

            @Override
            public void stop(boolean interrupted) {
                inner.stop(interrupted);
            }
        };
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


    int GOAL_TAG_ID = 0;

    // Tune these:

    public static double desiredTilt = 0;



    private AprilTagDetection d;


    public void periodic() {
        if (odometry != null) odometry.update();

        d = Webcam.INSTANCE.getDetectionById(GOAL_TAG_ID);

        // Always keep odometry-based estimate fresh when tag is visible
        if (d != null && d.ftcPose != null) {
            updateBlueTagEstimate(d);
        }
    }

    public void setGoalID(int id) {
        GOAL_TAG_ID = id;
    }

    private void syncFollowerPoseToOdometry(Follower follower) {
        if (follower == null || odometry == null) return;

        Pose2D pose = odometry.getPosition();
        if (pose == null) return;

        double xIn = pose.getX(DistanceUnit.INCH);
        double yIn = pose.getY(DistanceUnit.INCH);

        double headingRad = 0;

        if (UsingOdemtryInsteadOfIMU) {
          headingRad =  pose.getHeading(AngleUnit.RADIANS);
        } else if ( imu != null) {
            headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        if (!Double.isFinite(xIn) || !Double.isFinite(yIn) || !Double.isFinite(headingRad)) return;

        headingRad = AngleUnit.normalizeRadians(headingRad);

        // Pedro headings are radians; Pose expects x/y + heading.
        follower.setStartingPose(new Pose(xIn, yIn, headingRad));

        // Optional but nice: ensures the follower internal state is refreshed immediately
        follower.update();
    }

    public Command faceBlueGoal(Follower follower, double tiltAngle) {

        Command inner =  new Command() {

            // --- IMU fallback state ---
            private boolean usingImuFallback = false;
            private double targetAbsRad = Double.NaN;
            private long startNanos = 0;

            @Override
            public @NotNull Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

            @Override
            public void start() {
                syncFollowerPoseToOdometry(follower);
                followerIsActive = true;

                // Refresh detection right now (periodic() also does this, but be explicit)
                d = Webcam.INSTANCE.getDetectionById(GOAL_TAG_ID);

                // If tag is visible OR user disabled fallback, keep the current Pedro behavior
                if ((d != null && d.ftcPose != null) || !useIMUFallbackWhenNoTag) {
                    usingImuFallback = false;

                    double targetRad = tiltAngle;
                    if (tiltAngle == 0) {
                        targetRad = calcFaceBlueGoalTargetRad(follower);
                    }

                    // Pedro turn uses a relative angle (radians)
                    follower.turn(targetRad);
                    startNanos = System.nanoTime();
                    return;
                }

                // --- Tag NOT visible, fallback requested ---
                // We can only compute a heading if we have a world estimate.
                if (!haveTagEstimate || odometry == null || odometry.getPosition() == null || imu == null) {
                    usingImuFallback = false;

                    // fall back to whatever calcFaceBlueGoalTargetRad returns (even if it is NaN)
                    double targetRad = (tiltAngle == 0) ? calcFaceBlueGoalTargetRad(follower) : tiltAngle;
                    follower.turn(targetRad);
                    startNanos = System.nanoTime();
                    return;
                }

                usingImuFallback = true;
                startNanos = System.nanoTime();

                // Compute the ABSOLUTE target heading based on the stored tag estimate and current robot pose
                Pose2D pose = odometry.getPosition();
                double rx = pose.getX(DistanceUnit.INCH);
                double ry = pose.getY(DistanceUnit.INCH);

                double imuYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                imuYawRad = AngleUnit.normalizeRadians(imuYawRad);

                // Camera position (accounts for offsets)
                double camX = rx + camOffsetX_in * Math.cos(imuYawRad) - camOffsetY_in * Math.sin(imuYawRad);
                double camY = ry + camOffsetX_in * Math.sin(imuYawRad) + camOffsetY_in * Math.cos(imuYawRad);

                double dx = blueTagX_in - camX;
                double dy = blueTagY_in - camY;

                targetAbsRad = AngleUnit.normalizeRadians(
                        Math.atan2(dy, dx) - Math.toRadians(desiredTilt)
                );

                // Telemetry helpers (same variables you already show)
                lastFaceGoalAngleAbsolute = Math.toDegrees(targetAbsRad);
            }

            @Override
            public void update() {
                if (!usingImuFallback) {
                    follower.update(); // REQUIRED every loop for Pedro to actually run
                    return;
                }

                // --- IMU fallback: choose CW/CCW based on sign of shortest-path error ---
                double imuYawRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                imuYawRad = AngleUnit.normalizeRadians(imuYawRad);

                double errRad = AngleUnit.normalizeRadians(targetAbsRad - imuYawRad);
                lastFaceGoalAngleRelative = Math.toDegrees(errRad);

                // Stop if aligned
                if (d != null && d.ftcPose != null) {
                    stopDrive();
                    return;
                }

                // Turn in the direction of the error sign
                double turn = Math.copySign(imuFallbackTurnPower, errRad);

                // If your robot turns the wrong way, flip faceGoalTurnSign in dashboard (you already have this)
                turn *= -faceGoalTurnSign;

                setTurnPower(turn);
            }

            @Override
            public boolean isDone() {
                if (!usingImuFallback) {
                    return !follower.isBusy();
                }

                // Timeout OR within tolerance
                double elapsedSec = (System.nanoTime() - startNanos) / 1e9;

                if (elapsedSec >= imuFallbackTimeoutSec) return true;

                return (d != null && d.ftcPose != null);
            }

            @Override
            public void stop(boolean interrupted) {
                stopDrive();
                if (follower != null) follower.breakFollowing();
                followerIsActive = false;
            }
        };

        final boolean[] bearingDone = new boolean[1];
        bearingDone[0] = (d != null && d.ftcPose != null &&
                Math.abs(d.ftcPose.bearing) <= faceGoalBearingDoneDeg);

        return  new Command() {
            @Override
            public void start() {
                inner.start();
            }

            @Override
            public void update() {
                bearingDone[0] = (d != null && d.ftcPose != null &&
                        Math.abs(d.ftcPose.bearing) <= faceGoalBearingDoneDeg);
                    if (inner.isDone()) {
                        if (!(bearingDone[0]) ){
                            inner.start();
                        }
                    } else {
                        inner.update();
                    }
            }


            @Override
            public boolean isDone() {
                return  bearingDone[0] ;
            }

            @Override
            public void stop(boolean interrupted) {
                inner.stop(interrupted);
            }
        };
    }

    public double calcFaceBlueGoalTargetRad(Follower follower) {
        double deltaRad = Double.NaN;


        // --- Case 1: Tag visible -> just rotate by bearing (relative) ---
        // ftcPose.bearing is already a relative left/right angle to center the tag. :contentReference[oaicite:2]{index=2}
        if (d != null && d.ftcPose != null) {
            // Want bearing -> desiredTilt, so turn by (bearing - desiredTilt)
            double errRad = Math.toRadians(d.ftcPose.bearing - desiredTilt);
            errRad = AngleUnit.normalizeRadians(errRad);
            deltaRad = errRad * faceGoalTurnSign;

        }
        // --- Case 2: Tag not visible but we have world estimate -> compute heading error ---
        else if (haveTagEstimate && odometry != null && odometry.getPosition() != null && (imu != null || UsingOdemtryInsteadOfIMU)) {
            Pose2D pose = odometry.getPosition();

            double rx = pose.getX(DistanceUnit.INCH);
            double ry = pose.getY(DistanceUnit.INCH);


            double hRad = 0;

            if (UsingOdemtryInsteadOfIMU) {
                hRad =  pose.getHeading(AngleUnit.RADIANS);
            } else  if ( imu != null ) {
                hRad =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            if (!Double.isFinite(hRad)) {
                deltaRad = Double.NaN;

            } else {
                // Camera position (accounts for offsets)
                hRad = AngleUnit.normalizeRadians(hRad);
                double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
                double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

                double dx = blueTagX_in - camX;
                double dy = blueTagY_in - camY;


                double targetRad = Math.atan2(dy, dx) - Math.toRadians(desiredTilt);

                lastFaceGoalAngleAbsolute = Math.toDegrees(targetRad);

//              signed shortest-path error in [-pi, pi)
                deltaRad = AngleUnit.normalizeRadians(targetRad- follower.getPose().getHeading()) * faceGoalTurnSign;

            }
        }

        // Save + telemetry every call
        lastFaceGoalAngleRelative = Math.toDegrees(deltaRad);



        return deltaRad;
    }

    public  double getIMUHeading() {
        return  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }



}