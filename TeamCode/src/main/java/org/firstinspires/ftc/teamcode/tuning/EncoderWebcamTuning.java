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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;

@Config
@TeleOp(name = "Tuning - Encoders + Odo + Webcam", group = "Tuning")
public class EncoderWebcamTuning extends NextFTCOpMode {

    // ====== Hardware names (match Robot Config) ======
    public static String FRONT_LEFT_NAME  = "frontLeft";
    public static String FRONT_RIGHT_NAME = "frontRight";
    public static String BACK_LEFT_NAME   = "backLeft";
    public static String BACK_RIGHT_NAME  = "backRight";

    public static String INTAKE_NAME      = "Intake";
    public static String OUTTAKE_LEFT_NAME  = "flyWheelLeft";
    public static String OUTTAKE_RIGHT_NAME = "flyWheelRight";

    // New: odometry encoder (configure as a "motor" on the paired motor port)
    public static String ODOMETRY_NAME = "Odometry";

    public static String WEBCAM_NAME = "Webcam"; // or "Webcam 1" depending on your config

    // ====== Movement thresholds ======
    public static double MOVED_TICKS_THRESHOLD = 10;   // ticks
    public static double MOVED_VEL_THRESHOLD   = 20;   // ticks/sec
    public static long   RECENT_MOVE_MS        = 600;  // ms

    // Motors (NextFTC wrappers)
    private MotorEx fl, fr, bl, br;
    private MotorEx intake;
    private MotorEx outL, outR;

    // New: odometry "motor" encoder reader
    private MotorEx odo;

    // Simple tracking
    private MotorDiag[] motorDiags;
    private MotorDiag odoDiag;

    // Drive command (your existing drivetrain teleop)
    private Command driverControlled;

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

        // Odometry encoder reader (configured as a "motor" in config)
        odo = new MotorEx(ODOMETRY_NAME);

        motorDiags = new MotorDiag[] {
                new MotorDiag("FL", fl),
                new MotorDiag("FR", fr),
                new MotorDiag("BL", bl),
                new MotorDiag("BR", br),
                new MotorDiag("INT", intake),
                new MotorDiag("OUT_L", outL),
                new MotorDiag("OUT_R", outR),
        };

        odoDiag = new MotorDiag("ODO", odo);

        // A: software-zero all encoder readings
        gp1.getA().setPressedCommand(() -> { zeroAll(); return null; });

        // keep your existing toggles
        gp1.getB().setPressedCommand(() -> Intake.INSTANCE.eat());
        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.spit());
        gp1.getRightBumper().setPressedCommand(() -> Outtake.INSTANCE.handleMotor(Outtake.motorVelocityTarget));

        initVision();
    }

    @Override
    public void onUpdate() {
        long now = System.currentTimeMillis();

        telemetry.addLine("ENCODER + ODO + WEBCAM TUNER");
        telemetry.addLine("Drive around / run mechanisms. Watch ticks & vel change.");
        telemetry.addLine("A = zero encoder offsets");
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

            // Consider drivetrain as "robot is moving" if any drive motor is active
            if (d.label.equals("FL") || d.label.equals("FR") || d.label.equals("BL") || d.label.equals("BR")) {
                if (d.isRecentlyMoving(now)) driveRecentlyMoved = true;
            }
        }

        telemetry.addLine("");

        // Odometry encoder
        odoDiag.update(now);
        telemetry.addData(
                odoDiag.label,
                String.format(Locale.US,
                        "pos=%.0f  vel=%.0f  Δ=%.0f  [%s]",
                        odoDiag.pos, odoDiag.vel, odoDiag.delta, odoDiag.status(now))
        );

        // Odometry health rule: if robot is moving but ODO isn't changing, something's wrong
        if (driveRecentlyMoved && !odoDiag.isRecentlyMoving(now)) {
            telemetry.addLine("⚠ ODO WARNING: drivetrain encoders moved, but odometry did not.");
            telemetry.addLine("  Check: correct motor-port pairing, encoder cable, and config name.");
        } else {
            telemetry.addLine("ODO check: OK (or robot stationary).");
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

    private void zeroAll() {
        long now = System.currentTimeMillis();
        if (motorDiags != null) for (MotorDiag d : motorDiags) d.zero(now);
        if (odoDiag != null) odoDiag.zero(now);
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

    // ===== Helper class for compact encoder health reporting (with software zero offset) =====
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
}
