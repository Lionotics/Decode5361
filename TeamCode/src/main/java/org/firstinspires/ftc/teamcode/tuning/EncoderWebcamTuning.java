package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;

// ===== Pinpoint imports (I2C odometry computer) =====
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@TeleOp(name = "Tuning - Encoders + Pinpoint(Odo) + Webcam", group = "Tuning")
public class EncoderWebcamTuning extends NextFTCOpMode {

    // ====== Hardware names (match Robot Config) ======
    public static String FRONT_LEFT_NAME  = "frontLeft";
    public static String FRONT_RIGHT_NAME = "frontRight";
    public static String BACK_LEFT_NAME   = "backLeft";
    public static String BACK_RIGHT_NAME  = "backRight";

    public static String INTAKE_NAME      = "Intake";
    public static String OUTTAKE_LEFT_NAME  = "flyWheelLeft";
    public static String OUTTAKE_RIGHT_NAME = "flyWheelRight";

    // IMPORTANT: this is the *I2C Pinpoint device name* in the Robot Config
    public static String ODOMETRY_NAME = "Odometry";   // many examples use "odo" :contentReference[oaicite:6]{index=6}

    public static String WEBCAM_NAME = "Webcam"; // or "Webcam 1" depending on your config

    // ====== Movement thresholds ======
    public static double MOVED_TICKS_THRESHOLD = 10;   // ticks
    public static double MOVED_VEL_THRESHOLD   = 20;   // ticks/sec
    public static long   RECENT_MOVE_MS        = 600;  // ms

    // Pinpoint “is it moving?” thresholds (pose is in mm, vel in mm/sec)
    public static double ODO_MOVED_MM_THRESHOLD = 2.0;     // mm
    public static double ODO_MOVED_VEL_MMPS     = 5.0;     // mm/sec

    // Pinpoint setup (you can tune later; for “is it alive?” these can be 0)
    public static double PINPOINT_X_OFFSET_MM = 0.0;
    public static double PINPOINT_Y_OFFSET_MM = 0.0;

    // If you’re using goBILDA pods, pick the right one (4-bar vs swingarm).
    // Defaulting to 4-bar because it's common; change if needed.
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public static GoBildaPinpointDriver.EncoderDirection X_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection Y_DIR =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Motors (NextFTC wrappers)
    private MotorEx fl, fr, bl, br;
    private MotorEx intake;
    private MotorEx outL, outR;

    // Simple tracking
    private MotorDiag[] motorDiags;

    // Drive command (your existing drivetrain teleop)
    private Command driverControlled;

    // ===== Pinpoint (I2C odometry computer) =====
    private GoBildaPinpointDriver pinpoint;
    private PinpointDiag pinpointDiag;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private String cameraInitError = null;

    public EncoderWebcamTuning() {
        super(DriveTrain.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, Outtake.INSTANCE);
    }

    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gp1 = gamepadManager.getGamepad1();

        // Drive
        driverControlled = DriveTrain.INSTANCE.Drive(gp1, false);
        driverControlled.invoke();

        // Motor encoder readers
        fl = new MotorEx(FRONT_LEFT_NAME);
        fr = new MotorEx(FRONT_RIGHT_NAME);
        bl = new MotorEx(BACK_LEFT_NAME);
        br = new MotorEx(BACK_RIGHT_NAME);

        intake = new MotorEx(INTAKE_NAME);

        outL = new MotorEx(OUTTAKE_LEFT_NAME);
        outR = new MotorEx(OUTTAKE_RIGHT_NAME);

        motorDiags = new MotorDiag[] {
                new MotorDiag("FL", fl),
                new MotorDiag("FR", fr),
                new MotorDiag("BL", bl),
                new MotorDiag("BR", br),
                new MotorDiag("INT", intake),
                new MotorDiag("OUT_L", outL),
                new MotorDiag("OUT_R", outR),
        };

        // A: software-zero all motor encoder readings (your original behavior)
        gp1.getA().setPressedCommand(() -> { zeroAllMotors(); return null; });

        // Keep your existing toggles
        gp1.getB().setPressedCommand(() -> Intake.INSTANCE.eat());
        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.spit());
        gp1.getRightBumper().setPressedCommand(() -> Outtake.INSTANCE.handleMotor(Outtake.motorVelocityTarget));

        // ===== Init Pinpoint =====
        initPinpoint(gp1);

        // Vision
        initVision();
    }

    @Override
    public void onUpdate() {
        long now = System.currentTimeMillis();

        telemetry.addLine("ENCODER + PINPOINT(ODO) + WEBCAM TUNER");
        telemetry.addLine("Drive around / run mechanisms. Watch ticks & Pinpoint pose/vel change.");
        telemetry.addLine("A = zero motor encoder offsets");
        telemetry.addLine("X = Pinpoint resetPosAndIMU (robot must be STILL)");
        telemetry.addLine("");

        // Drive/motor encoders
        boolean driveRecentlyMoved = false;
        for (MotorDiag d : motorDiags) {
            d.update(now);
            telemetry.addData(
                    d.label,
                    String.format(Locale.US,
                            "pos=%.0f  vel=%.0f  Δ=%.0f  [%s]",
                            d.pos, d.vel, d.delta, d.status(now))
            );

            if (d.label.equals("FL") || d.label.equals("FR") || d.label.equals("BL") || d.label.equals("BR")) {
                if (d.isRecentlyMoving(now)) driveRecentlyMoved = true;
            }
        }

        telemetry.addLine("");

        // ===== Pinpoint diagnostics =====
        if (pinpoint == null) {
            telemetry.addLine("Pinpoint: NOT INITIALIZED (check I2C config name/type + driver install)");
        } else {
            // Must call update() to pull new data :contentReference[oaicite:7]{index=7}
            pinpoint.update();

            // Pose/Velocity
            Pose2D pos = pinpoint.getPosition();
            // Units from Pinpoint: mm/sec and rad/sec (per goBILDA docs)
           double vx = pinpoint.getVelX(DistanceUnit.MM);          // mm/s
            double vy = pinpoint.getVelY(DistanceUnit.MM);          // mm/s
            double vhRad = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);       // rad/s
            double vhDeg = AngleUnit.RADIANS.toDegrees(vhRad);

            telemetry.addData("Pinpoint vel (mm/s,deg/s)",
                    "{VX:%.1f VY:%.1f VH:%.1f}", vx, vy, vhDeg);
                    ;

            double x = pos.getX(DistanceUnit.MM);
            double y = pos.getY(DistanceUnit.MM);
            double h = pos.getHeading(AngleUnit.DEGREES);



            if (pinpointDiag != null) pinpointDiag.update(now, x, y, h, vx, vy, vhDeg);

            telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus()); // READY / FAULT_* etc :contentReference[oaicite:8]{index=8}
            telemetry.addData("Pinpoint freq (Hz)", "%.0f", pinpoint.getFrequency());
            telemetry.addData("Pinpoint pose (mm/deg)", "{X:%.1f Y:%.1f H:%.1f}", x, y, h);
            telemetry.addData("Pinpoint vel (mm/s,deg/s)", "{VX:%.1f VY:%.1f VH:%.1f}", vx, vy, vhDeg);

            // Health rule: if robot is moving (drive encoders), but Pinpoint isn't changing -> warn
            boolean pinpointMoving = (pinpointDiag != null) && pinpointDiag.isRecentlyMoving(now);
            if (driveRecentlyMoved && !pinpointMoving) {
                telemetry.addLine("⚠ ODO WARNING: drivetrain encoders moved, but Pinpoint pose/vel did not.");
                telemetry.addLine("  Check: Pinpoint name/type on I2C, NOT on I2C port 0, pods plugged into Pinpoint.");
            } else {
                telemetry.addLine("ODO check: OK (or robot stationary).");
            }
        }

        telemetry.addLine("");

        // Webcam diagnostics
        if (cameraInitError != null) {
            telemetry.addData("Webcam", "ERROR: %s", cameraInitError);
        } else if (visionPortal == null) {
            telemetry.addData("Webcam", "Not initialized");
        } else {
            telemetry.addData("Camera state", visionPortal.getCameraState());
            telemetry.addData("Camera FPS", "%.1f", visionPortal.getFps());

            int n = (aprilTag != null && aprilTag.getDetections() != null) ? aprilTag.getDetections().size() : 0;
            telemetry.addData("AprilTag detections", n);

            if (n > 0) {
                AprilTagDetection det = aprilTag.getDetections().get(0);
                telemetry.addData("Tag[0] id", det.id);
                telemetry.addData("Tag[0] range (in)", "%.1f", det.ftcPose.range);
                telemetry.addData("Tag[0] bearing (deg)", "%.1f", det.ftcPose.bearing);
            } else {
                telemetry.addLine("Tip: point camera at an AprilTag to see id/range/bearing.");
            }
        }

        telemetry.update();
    }

    private void zeroAllMotors() {
        long now = System.currentTimeMillis();
        if (motorDiags != null) for (MotorDiag d : motorDiags) d.zero(now);
    }

    private void initPinpoint(GamepadEx gp1) {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_NAME);

            // Minimal setup; required for “correct” pose, but not strictly required to prove “alive”
            pinpoint.setOffsets(PINPOINT_X_OFFSET_MM, PINPOINT_Y_OFFSET_MM, DistanceUnit.MM); // mm :contentReference[oaicite:9]{index=9}
            pinpoint.setEncoderResolution(POD_TYPE);                         // goBILDA pod preset :contentReference[oaicite:10]{index=10}
            pinpoint.setEncoderDirections(X_DIR, Y_DIR);                     // ensure +X forward, +Y left :contentReference[oaicite:11]{index=11}

            // Calibrate/zero at init (robot must be stationary!) :contentReference[oaicite:12]{index=12}
            pinpoint.resetPosAndIMU();

            pinpointDiag = new PinpointDiag();

            // X: re-zero + recalibrate IMU (robot must be STILL)
            gp1.getX().setPressedCommand(() -> { pinpoint.resetPosAndIMU(); return null; });

        } catch (Exception e) {
            pinpoint = null;
            pinpointDiag = null;
            telemetry.addData("Pinpoint init", "ERROR: %s: %s", e.getClass().getSimpleName(), e.getMessage());
        }
    }

    private void initVision() {
        try {
            aprilTag = new AprilTagProcessor.Builder().build();
            WebcamName cam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
            visionPortal = VisionPortal.easyCreateWithDefaults(cam, aprilTag);
        } catch (Exception e) {
            cameraInitError = e.getClass().getSimpleName() + ": " + e.getMessage();
            visionPortal = null;
            aprilTag = null;
        }
    }

    // ===== Helper class for compact motor encoder health reporting (with software zero offset) =====
    private static class MotorDiag {
        final String label;
        final MotorEx motor;

        double pos = 0;      // zeroed position
        double vel = 0;
        double lastPos = 0;
        double delta = 0;

        double zeroOffset = 0;

        long lastMoveMs = 0;
        boolean everMoved = false;

        MotorDiag(String label, MotorEx motor) {
            this.label = label;
            this.motor = motor;
            zero(System.currentTimeMillis());
        }

        void zero(long now) {
            zeroOffset = motor.getCurrentPosition();
            pos = 0;
            vel = motor.getVelocity();
            lastPos = 0;
            delta = 0;
            lastMoveMs = now;
            everMoved = false;
        }

        void update(long now) {
            double raw = motor.getCurrentPosition();
            pos = raw - zeroOffset;

            vel = motor.getVelocity();
            delta = pos - lastPos;
            lastPos = pos;

            if (Math.abs(delta) >= MOVED_TICKS_THRESHOLD || Math.abs(vel) >= MOVED_VEL_THRESHOLD) {
                lastMoveMs = now;
                everMoved = true;
            }
        }

        boolean isRecentlyMoving(long now) {
            return everMoved && (now - lastMoveMs <= RECENT_MOVE_MS);
        }

        String status(long now) {
            if (!everMoved) return "NOT MOVED YET";
            if (now - lastMoveMs <= RECENT_MOVE_MS) return "ACTIVE";
            return "OK (IDLE)";
        }
    }

    // ===== Pinpoint motion detector (simple “is pose/vel changing?” tracker) =====
    private class PinpointDiag {
        double lastX = 0, lastY = 0, lastH = 0;
        long lastMoveMs = 0;
        boolean everMoved = false;

        void update(long now, double x, double y, double hDeg, double vx, double vy, double vhDeg) {
            double dx = x - lastX;
            double dy = y - lastY;

            lastX = x;
            lastY = y;
            lastH = hDeg;

            double distDelta = Math.hypot(dx, dy);
            double speed = Math.hypot(vx, vy);

            if (distDelta >= ODO_MOVED_MM_THRESHOLD || speed >= ODO_MOVED_VEL_MMPS) {
                lastMoveMs = now;
                everMoved = true;
            }
        }

        boolean isRecentlyMoving(long now) {
            return everMoved && (now - lastMoveMs <= RECENT_MOVE_MS);
        }
    }
}
