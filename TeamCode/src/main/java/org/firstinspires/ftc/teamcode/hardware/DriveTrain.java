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






    // If true: faceGoal aims at the BACK CORNER of the goal (using tag pose + geometry),
// not "center tag bearing = desiredBearing".
// If false: behavior is exactly as before.
    public static boolean alignToGoalInsteadOfTag = false;

    // Corner offset relative to the TAG CENTER, expressed in the TAG'S LOCAL 2D frame (inches).
// You will tune these on FTC Dashboard.
// Sign convention depends on how your tag yaw behaves; start small and tune.
    public static double blueGoalCornerOffsetX_in = 0;
    public static double blueGoalCornerOffsetY_in = 11.0;

    public static double redGoalCornerOffsetX_in  = 0;
    public static double redGoalCornerOffsetY_in  = 11.0;

    // If yaw is noisy, you can disable using yaw in the corner math.
    public static boolean useTagYawForCorner = true;







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
    public static double imuFallbackTurnPower = 0.2;
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


    public  static  double desiredBearingCloserBlue = 0;
    public  static  double desiredBearingCloserRed = 0;

    public  static  double desiredBearingFartherBlue = -2;

    public  static  double desiredBearingFartherRed = 2;

    public  static  double desiredBearingDistanceDivider = 90;





    private AprilTagDetection d;



    public void periodic() {
        if (odometry != null) odometry.update();



        if (Webcam.shouldUpdateCameraNow()) {
            d = Webcam.INSTANCE.getDetectionById(GOAL_TAG_ID);
        } else {
            d = null;
        }


            // Always keep odometry-based estimate fresh when tag is visible
            if (d != null && d.ftcPose != null) {
                updateBlueTagEstimate(d);
            }

    }

    public  double getDesiredBearing() {
        if (Webcam.INSTANCE.getRange() > desiredBearingDistanceDivider) {
            if (GOAL_TAG_ID == 24) {
                return  desiredBearingFartherRed;
            } else {
                return  desiredBearingFartherBlue;
            }

        } else {
            if (GOAL_TAG_ID == 24) {
                return  desiredBearingCloserRed;
            } else {
                return  desiredBearingCloserBlue;
            }
        }

    }


    private double computeGoalCornerBearingDeg(AprilTagDetection det) {
        if (det == null || det.ftcPose == null) return Double.NaN;

        // --- 1) Tag center position in camera 2D (floor-projected) ---
        // FTC docs: "range" is point-to-point; projecting onto the floor with cos(elevation)
        // is often more consistent if the camera is above the tag. :contentReference[oaicite:2]{index=2}
        double bearingRad = Math.toRadians(det.ftcPose.bearing);
        double elevRad    = Math.toRadians(det.ftcPose.elevation);

        elevRad = 0;

        double horizontalRange = det.ftcPose.range * Math.cos(elevRad);

        double forward = horizontalRange * Math.cos(bearingRad);
        double left    = horizontalRange * Math.sin(bearingRad);

        // --- 2) Pick the correct offsets for red vs blue ---
        double offsetX = redGoalCornerOffsetX_in;
        double offsetY = redGoalCornerOffsetY_in;

        if (det.id == 20) {
            offsetX = blueGoalCornerOffsetX_in;
            offsetY = blueGoalCornerOffsetY_in;
        } else if (det.id == 24) {
            offsetX = redGoalCornerOffsetX_in;
            offsetY = redGoalCornerOffsetY_in;
        } else {
            // Unknown tag id: fall back to "red" constants (or change as you prefer)
            offsetX = redGoalCornerOffsetX_in;
            offsetY = redGoalCornerOffsetY_in;
        }

        // Tag-local offsets expressed as (forward_from_tag, left_from_tag)
        // "farther/back" => +forward
        // "right"        => -left
        double offForward_tag = offsetY;
        double offLeft_tag    = -offsetX;

        // --- 3) Rotate offsets by tag yaw (optional) and add to tag center ---
        // FTC yaw is rotation about Z, right-hand rule.
        if (useTagYawForCorner) {
            double yawRad = Math.toRadians(det.ftcPose.yaw);

            double offForward_cam = offForward_tag * Math.cos(yawRad) - offLeft_tag * Math.sin(yawRad);
            double offLeft_cam    = offForward_tag * Math.sin(yawRad) + offLeft_tag * Math.cos(yawRad);

            forward += offForward_cam;
            left    += offLeft_cam;
        } else {
            // If yaw is noisy, assume tag is "facing" the camera (no rotation).
            forward += offForward_tag;
            left    += offLeft_tag;
        }

        // --- 4) Bearing to the corner point ---
        // atan2(left, forward) gives the left/right turn angle needed to point at that point.
        return Math.toDegrees(Math.atan2(left, forward));
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

    public Command
    faceGoal(Follower follower, double tiltAngle) {
         final boolean[] usingImuFallback  = new boolean[1];
              usingImuFallback[0] =    false;



        Command inner =  new Command() {



            // --- IMU fallback state ---
            private long startNanos = 0;


            @Override
            public @NotNull Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

            @Override
            public void start() {
                Webcam.beginCameraUse();
                syncFollowerPoseToOdometry(follower);
                followerIsActive = true;


                d = Webcam.INSTANCE.getDetectionById(GOAL_TAG_ID);



                // If tag is visible OR user disabled fallback, keep the current Pedro behavior
                if ((d != null && d.ftcPose != null) || !useIMUFallbackWhenNoTag) {
                    usingImuFallback[0] = false;

                    double targetRad = tiltAngle;
                    if (tiltAngle == 0) {
                        targetRad = calcFaceBlueGoalTargetRad(follower);
                    }

                    // Pedro turn uses a relative angle (radians)
                    follower.turn(targetRad);
                    startNanos = System.nanoTime();
                    return;
                }


                usingImuFallback[0] = true;
                startNanos = System.nanoTime();






            }

            @Override
            public void update() {
                if (!usingImuFallback[0]) {
                    follower.update(); // REQUIRED every loop for Pedro to actually run
                    return;
                }


                // Stop if aligned
                if (d != null && d.ftcPose != null) {
                    stopDrive();
                    return;
                }


                // If your robot turns the wrong way, flip faceGoalTurnSign in dashboard (you already have this)
                double turn = imuFallbackTurnPower * faceGoalTurnSign;

                setTurnPower(turn);
            }

            @Override
            public boolean isDone() {
                if (!usingImuFallback[0]) {
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
                Webcam.endCameraUse();
            }
        };

        final boolean[] bearingDone = new boolean[1];
        bearingDone[0] = ( inner.isDone() &&   (!usingImuFallback[0] || (d != null && d.ftcPose != null &&
                Math.abs(d.ftcPose.bearing) <= faceGoalBearingDoneDeg) )) ;

        Command outer =   new Command() {
            @Override
            public void start() {
                inner.start();
            }

            @Override
            public @NotNull Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

            @Override
            public void update() {
                bearingDone[0] = ( inner.isDone() &&   (!usingImuFallback[0] || (d != null && d.ftcPose != null &&
                        Math.abs(d.ftcPose.bearing) <= faceGoalBearingDoneDeg) )) ;
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
                bearingDone[0] = ( inner.isDone() &&   (!usingImuFallback[0] || (d != null && d.ftcPose != null &&
                        Math.abs(d.ftcPose.bearing) <= faceGoalBearingDoneDeg) )) ;
                return  bearingDone[0] ;
            }

            @Override
            public void stop(boolean interrupted) {
                inner.stop(interrupted);
            }
        };

        return  outer;
    }

    public double calcFaceBlueGoalTargetRad(Follower follower) {
        double deltaRad = Double.NaN;



        // --- Case 1: Tag visible -> rotate by bearing (relative) ---
        if (d != null && d.ftcPose != null) {

            double measuredBearingDeg;

            if (alignToGoalInsteadOfTag) {
                // Use computed bearing to goal CORNER (tag pose only; no odometry x/y).
                measuredBearingDeg = computeGoalCornerBearingDeg(d);
            } else {
                // Original behavior: use tag center bearing
                measuredBearingDeg = d.ftcPose.bearing;
            }

            double desiredBearing = getDesiredBearing(); // your existing distance-based bias

            double errRad = Math.toRadians(measuredBearingDeg - desiredBearing);
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


                double desiredBearing = getDesiredBearing();

                double targetRad = Math.atan2(dy, dx) - Math.toRadians(desiredBearing);

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