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


    public static double lastFaceBlueGoalAngle = Double.NaN;

    public  boolean followerIsActive = false;


    // If your robot turns the wrong direction, set this to -1 in dashboard.
    public static double faceGoalTurnSign = 1.0;

    public  static  boolean UsingOdemtryInsteadOfIMU = true; // if true, using odemtry, if false, using IMU




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

        double horizontalRange = d.ftcPose.range * Math.cos(elevRad);

       // horizontalRange = d.ftcPose.range; // uncomment this code if accounting for elevation causes problems



        double measTagX = camX + horizontalRange * Math.cos(dirRad);
        double measTagY = camY + horizontalRange * Math.sin(dirRad);

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

        double headingDeg = 0;

        if (UsingOdemtryInsteadOfIMU) {
          headingDeg =  pose.getHeading(AngleUnit.DEGREES);
        } else if ( imu != null) {
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
                    double targetDeg = tiltAngle;  // your method returns DEGREES

                    if (tiltAngle == 0) {
                        targetDeg = calcFaceBlueGoalTargetDeg();
                    }


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
        double deltaDeg = Double.NaN;

        if (odometry != null) odometry.update();

        // --- Case 1: Tag visible -> just rotate by bearing (relative) ---
        // ftcPose.bearing is already a relative left/right angle to center the tag. :contentReference[oaicite:2]{index=2}
        if (d != null && d.ftcPose != null) {
            // Want bearing -> desiredTilt, so turn by (bearing - desiredTilt)
            deltaDeg = wrapDeg((d.ftcPose.bearing - desiredTilt) * faceGoalTurnSign);
        }
        // --- Case 2: Tag not visible but we have world estimate -> compute heading error ---
        else if (haveTagEstimate && odometry != null && odometry.getPosition() != null && imu != null) {
            Pose2D pose = odometry.getPosition();

            double rx = pose.getX(DistanceUnit.INCH);
            double ry = pose.getY(DistanceUnit.INCH);

            double hDeg = 0;

            if (UsingOdemtryInsteadOfIMU) {
                hDeg =  pose.getHeading(AngleUnit.DEGREES);
            } else  if ( imu != null ) {
                hDeg =  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }

            if (!Double.isFinite(hDeg)) {
                deltaDeg = Double.NaN;

            } else {
                // Camera position (accounts for offsets)
                double hRad = Math.toRadians(hDeg);
                double camX = rx + camOffsetX_in * Math.cos(hRad) - camOffsetY_in * Math.sin(hRad);
                double camY = ry + camOffsetX_in * Math.sin(hRad) + camOffsetY_in * Math.cos(hRad);

                double dx = blueTagX_in - camX;
                double dy = blueTagY_in - camY;

                // Absolute direction to tag
                double dirToTagDeg = Math.toDegrees(Math.atan2(dy, dx));

                // Desired robot heading so camera points at tag (minus desired tilt)
                double headingTargetDeg = wrapDeg(dirToTagDeg - desiredTilt);

                // RELATIVE turn needed from current heading
                deltaDeg = wrapDeg((headingTargetDeg - hDeg) * faceGoalTurnSign);
            }
        }

        // Save + telemetry every call
        lastFaceBlueGoalAngle = deltaDeg;



        return deltaDeg;
    }

    public  double getIMUHeading() {
        return  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }



}