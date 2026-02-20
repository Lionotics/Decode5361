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




    public  boolean drivingFieldCentricNoTurnIsActivated = false;


    // --- debug telemetry for faceBlueGoalPedro ---
    public static double faceGoal_targetHeadingDeg = Double.NaN;
    public static double faceGoal_dirToTagDeg = Double.NaN;
    public static double faceGoal_liveBearingDeg = Double.NaN;
    public static String faceGoal_source = "NONE";

    public static double faceGoal_lastTargetHeadingDeg = Double.NaN;  // "tilt to face goal since last check"
    public static long   faceGoal_lastComputedAtMs = 0;

    public  boolean followerIsActive = false;




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
        //double hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); // we use Imu for heading since using pose caused some problem with MechnamControlled


        // Camera position in world frame (approx)
        double hRad = Math.toRadians(hDeg);
        double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
        double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

        // Direction from camera to tag in world frame
        double dirDeg = hDeg + d.ftcPose.bearing;   // bearing positive = camera must turn left/right :contentReference[oaicite:5]{index=5}
        double dirRad = Math.toRadians(dirDeg);

        double measTagX = camX + d.ftcPose.range * Math.cos(dirRad);
        double measTagY = camY + d.ftcPose.range * Math.sin(dirRad);

        if (!haveTagEstimate) {
            blueTagX_in = measTagX;
            blueTagY_in = measTagY;
            haveTagEstimate = true;
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

        drivingFieldCentricNoTurnIsActivated = false;

        Webcam.INSTANCE.setSoleTagID(GOAL_TAG_ID);

        followerIsActive = false;
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
                drivingFieldCentricNoTurnIsActivated = false;
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

        odometry.update();
        Pose2D pose = odometry.getPosition();
        if (pose == null) return;

        double xIn = pose.getX(DistanceUnit.INCH);
        double yIn = pose.getY(DistanceUnit.INCH);

        double headingDeg = pose.getHeading(AngleUnit.DEGREES);
        if (!Double.isFinite(headingDeg) && imu != null) {
            headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        if (!Double.isFinite(xIn) || !Double.isFinite(yIn) || !Double.isFinite(headingDeg)) return;

        // Pedro headings are radians; Pose expects x/y + heading.
        follower.setStartingPose(new Pose(xIn, yIn, Math.toRadians(headingDeg)));

        // Optional but nice: ensures the follower internal state is refreshed immediately
        follower.update();
    }

    public Command faceBlueGoal(Follower follower, double tiltAngle) {
        return new LambdaCommand()
                .setSubsystems(this) // claims DriveTrain so normal driving won’t fight it
                .setStart(() -> {
                    followerIsActive = true;
                    double targetDeg = tiltAngle; //calcFaceBlueGoalTargetDeg(); // your method returns DEGREES
                    syncFollowerPoseToOdometry(follower);
                    if (Double.isFinite(targetDeg)) {
                        follower.turn(Math.toRadians(targetDeg));  // absolute “turn to heading”
                    }
                })
                .setUpdate(() -> {
                    follower.update(); // REQUIRED every loop for Pedro to actually run
                })
                .setIsDone(() -> !follower.isBusy())
                .setStop(interrupted -> {
                    // Optional: you can stop motors or just let TeleOp drive resume afterwards
                   stopDrive();
                   follower.breakFollowing();
                    followerIsActive = false;

                });
    }

    public double calcFaceBlueGoalTargetDeg() {

        faceGoal_source = "NONE";
        faceGoal_lastTargetHeadingDeg = Double.NaN;
        faceGoal_dirToTagDeg = Double.NaN;
        faceGoal_liveBearingDeg = Double.NaN;

        // Always grab a fresh detection (don’t rely on periodic timing)
        AprilTagDetection det = Webcam.INSTANCE.getDetectionById(GOAL_TAG_ID);

        // Update odometry pose
        if (odometry != null) odometry.update();
        Pose2D pose = (odometry != null) ? odometry.getPosition() : null;
        if (pose == null) {
            faceGoal_source = "NO_POSE";
            return Double.NaN;
        }

        // If Pinpoint heading ever glitches, use IMU as fallback
        double hDeg = pose.getHeading(AngleUnit.DEGREES);
        if (!Double.isFinite(hDeg) && imu != null) {
            hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        if (!Double.isFinite(hDeg)) {
            faceGoal_source = "BAD_HEADING";
            return Double.NaN;
        }

        // LIVE TAG branch (only if bearing is valid)
        if (det != null && det.ftcPose != null && Double.isFinite(det.ftcPose.bearing)) {
            faceGoal_source = "LIVE_TAG";
            faceGoal_liveBearingDeg = det.ftcPose.bearing;

            // Direction from robot to tag in FIELD frame (deg)
            // (heading + bearing) points from robot toward tag.
            faceGoal_dirToTagDeg = wrapDeg(hDeg + det.ftcPose.bearing);

            // Heading that makes bearing -> desiredTilt (usually 0)
            double targetHeadingDeg = wrapDeg(hDeg + (det.ftcPose.bearing - desiredTilt));

            faceGoal_lastTargetHeadingDeg = targetHeadingDeg;
            faceGoal_lastComputedAtMs = System.currentTimeMillis();
            return targetHeadingDeg;
        }

        // Keep estimate fresh if pose exists but bearing is invalid this frame
        if (det != null && det.ftcPose != null) {
            updateBlueTagEstimate(det);
        }

        // ODOMETRY ESTIMATE fallback
        if (haveTagEstimate) {
            faceGoal_source = "ODO_ESTIMATE";

            double rx = pose.getX(DistanceUnit.INCH);
            double ry = pose.getY(DistanceUnit.INCH);

            double dx = blueTagX_in - rx;
            double dy = blueTagY_in - ry;

            faceGoal_dirToTagDeg = Math.toDegrees(Math.atan2(dy, dx));
            faceGoal_lastTargetHeadingDeg = wrapDeg(faceGoal_dirToTagDeg - desiredTilt);
            faceGoal_lastComputedAtMs = System.currentTimeMillis();
            return faceGoal_lastTargetHeadingDeg;
        }

        faceGoal_source = "NO_TAG_NO_ESTIMATE";
        faceGoal_lastTargetHeadingDeg = wrapDeg(hDeg);
        faceGoal_lastComputedAtMs = System.currentTimeMillis();
        return faceGoal_lastTargetHeadingDeg;
    }



    /*public Command drivingFieldCentricFacingGoal(GamepadEx gp1) {
        // --- Tuning knobs (you can @Config these if you want) ---
        final double stickDeadband = 0.04;
        final double maxTurnPower = 0.60;     // cap rotation so it doesn’t whip
        final double maxTurnErrorDeg = 45.0;  // for optional scaling / sanity
        final boolean turnAtAll[] = new boolean[1];
        turnAtAll[0] = true;


        // If your robot turns the wrong direction, flip this to -1.
        final double turnSign = 1.0;

        // --- Helpers ---
        java.util.function.DoubleUnaryOperator deadband = (v) ->
                (Math.abs(v) < stickDeadband) ? 0.0 : v;

        java.util.function.DoubleUnaryOperator clip = (v) ->
                Math.max(-1.0, Math.min(1.0, v));

        java.util.function.DoubleUnaryOperator wrapDeg = (deg) -> {
            double a = deg;
            while (a <= -180) a += 360;
            while (a > 180) a -= 360;
            return a;
        };

        // --- Axis reads (ONE of these styles will match your GamepadEx API) ---
        // Style A (FTCLib-like):
        java.util.function.DoubleSupplier driveIn = () -> gp1.getLeftStick().getY();   // forward is negative stick Y
        java.util.function.DoubleSupplier strafeIn = () ->  gp1.getLeftStick().getX();


        // If your GamepadEx does NOT have getLeftX/getLeftY/getRightX, swap to whatever your API is:
        // e.g. gp1.leftStickY.get(), gp1.leftStickX.get(), gp1.rightStickX.get(), etc.

        // --- Turn supplier: always face blue goal ---
        java.util.function.DoubleSupplier turnIn = () -> {

            // 1) Tag visible: use live bearing (same logic as your faceBlueGoal)
            if (d != null && d.ftcPose != null) {
                double errorDeg = wrapDeg(desiredTilt - d.ftcPose.bearing);
                if (Math.abs(errorDeg) < deadbandDeg ) {
                    turnAtAll[0] = false;
                    return  0.0;
                } {
                    turnAtAll[0] = true;
                }


                double turn = Range.clip(kP_inTag * errorDeg, -maxTurn_inTag, maxTurn_inTag);
                turn = signedMinPower(turn, minTurn_inTag);
                return turn;
            }

            // 2) Tag not visible: use your odometry-based tag estimate (blueTagX_in / blueTagY_in)
            if (haveTagEstimate && odometry != null) {
                Pose2D pose = odometry.getPosition();

                double rx = pose.getX(DistanceUnit.INCH);
                double ry = pose.getY(DistanceUnit.INCH);

                // double hDeg = pose.getHeading(AngleUnit.DEGREES);
                double hDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

                double dx = blueTagX_in - rx;
                double dy = blueTagY_in - ry;

                double dirToTagDeg = Math.toDegrees(Math.atan2(dy, dx));

                // Same derivation you used in faceBlueGoal:
                // headingTarget ≈ dirToTag - desiredTilt
                double headingTargetDeg = dirToTagDeg - desiredTilt;

                double errorDeg = wrapDeg(headingTargetDeg - hDeg);
                errorDeg = -errorDeg; // keep your sign convention consistent

                double turn = Range.clip(kP_noTag * errorDeg, -maxTurn_noTag, maxTurn_noTag);
                turn = signedMinPower(turn, minTurn_noTag);
                return turn;
            }

            // 3) No estimate yet: gentle search turn
            return searchPowerBeforeTag;
        };

        // --- Build a field-centric MecanumDriverControlled using suppliers ---
        MecanumDriverControlled driverControlled = new MecanumDriverControlled(
                motors,
                () -> (float) clip.applyAsDouble(deadband.applyAsDouble(driveIn.getAsDouble())),
                () -> (float) clip.applyAsDouble(deadband.applyAsDouble(strafeIn.getAsDouble())),
                () -> {

                    double autoTurn = turnIn.getAsDouble();

                    return (float) clip.applyAsDouble(autoTurn);
                },
                false,   // robotCentric = false => FIELD CENTRIC
                imu      // IMU used for field-centric transform
        );

        // Match your existing pattern of “wrap a command so DriveTrain is a requirement”
        return new Command() {
            @Override public void start() { driverControlled.start(); }
            @Override public void update() {
                if (true ) {
                    driverControlled.update();
                }
            }
            @Override public void stop(boolean interrupted) { driverControlled.stop(interrupted); }
            @Override public boolean isDone() { return false; }
            @Override public Set<Subsystem> getSubsystems() {
                return Collections.singleton(DriveTrain.this);
            }

        };
    } */



}