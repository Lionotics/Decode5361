package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp(name = "PP: XY Velocity Tuner (NextFTCOpMode)", group = "Tuning")
public class XYVelocityTuner extends NextFTCOpMode {

    // These match YOUR DriveTrain.java swap:
    // frontLeft object uses hardware name "backLeft", etc. :contentReference[oaicite:4]{index=4}
    private static final String FL_NAME = "backLeft";
    private static final String FR_NAME = "backRight";
    private static final String BL_NAME = "frontLeft";
    private static final String BR_NAME = "frontRight";

    // Pinpoint device name in the RC config (matches your code) :contentReference[oaicite:5]{index=5}
    private static final String PINPOINT_NAME = "Odometry";

    // ===== TUNER SETTINGS =====
    public static  double TEST_DISTANCE_IN = 12.0; // 0.5 tiles
    private static final double MAX_POWER = 1.0;
    private static final double RAMP_SECONDS = 0.60;
    private static final double TIMEOUT_SECONDS = 8.0;
    private static final double PAUSE_SECONDS = 0.60;
    private static final double IMU_RESET_STILL_SECONDS = 0.30; // let Pinpoint calibrate
    // =========================

    private DcMotorEx fl, fr, bl, br;
    private GoBildaPinpointDriver odo;

    private final ElapsedTime timer = new ElapsedTime();
    private double startXmm = 0, startYmm = 0;

    private double maxAbsVxIn = 0.0; // forward axis max (your xVelocity)
    private double maxAbsVyIn = 0.0; // strafe axis max  (your yVelocity)

    private enum Phase { INIT, CALIBRATE_AFTER_START, FORWARD, PAUSE1, STRAFE, DONE }
    private Phase phase = Phase.INIT;

    private static double mmToIn(double mm) { return mm / 25.4; }
    //private static double mmToIn(double mm) { return mm; }


    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, FL_NAME);
        fr = hardwareMap.get(DcMotorEx.class, FR_NAME);
        bl = hardwareMap.get(DcMotorEx.class, BL_NAME);
        br = hardwareMap.get(DcMotorEx.class, BR_NAME);

        // Match your DriveTrain: left side reversed :contentReference[oaicite:6]{index=6}
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx[] motors = new DcMotorEx[] { fl, fr, bl, br };
        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }

        odo = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        phase = Phase.INIT;
        timer.reset();
    }

    @Override
    public void onWaitForStart() {
        // Keep robot still while Pinpoint calibrates; resetPosAndIMU does IMU zeroing :contentReference[oaicite:7]{index=7}
        if (phase == Phase.INIT) {
            telemetry.addLine("KEEP ROBOT STILL: resetting Pinpoint pos+IMU...");
            telemetry.update();

            // Your DriveTrain sets encoder directions (REVERSED, FORWARD) :contentReference[oaicite:8]{index=8}
            odo.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            odo.resetPosAndIMU(); // must be stationary :contentReference[oaicite:9]{index=9}
            timer.reset();
            phase = Phase.CALIBRATE_AFTER_START; // we’ll just use this as a “ready” marker
        }

        odo.update();

        double vxIn = mmToIn(odo.getVelX(DistanceUnit.MM));
        double vyIn = mmToIn(odo.getVelY(DistanceUnit.MM));

        telemetry.addLine("INIT OK. When you press START, robot will:");
        telemetry.addLine("  1) Drive FORWARD 48 in, stop");
        telemetry.addLine("  2) Strafe RIGHT  48 in, stop");
        telemetry.addLine("");
        telemetry.addData("Live vx (in/s)", vxIn);
        telemetry.addData("Live vy (in/s)", vyIn);
        telemetry.addData("Note", "Pinpoint convention: +X forward, +Y left :contentReference[oaicite:10]{index=10}");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Recalibrate right at start too (best practice: still robot) :contentReference[oaicite:11]{index=11}
        stopDrive();
        odo.resetPosAndIMU();
        timer.reset();
        phase = Phase.CALIBRATE_AFTER_START;

        maxAbsVxIn = 0.0;
        maxAbsVyIn = 0.0;
    }

    @Override
    public void onUpdate() {
        odo.update();

        if (phase == Phase.CALIBRATE_AFTER_START) {
            // give Pinpoint a moment to finish the “must be stationary” calibration :contentReference[oaicite:12]{index=12}
            telemetry.addLine("Starting… keep robot still for a moment");
            telemetry.addData("Countdown (s)", Math.max(0, IMU_RESET_STILL_SECONDS - timer.seconds()));
            telemetry.update();

            if (timer.seconds() >= IMU_RESET_STILL_SECONDS) {
                // begin forward segment
                startXmm = odo.getPosX(DistanceUnit.MM);
                startYmm = odo.getPosY(DistanceUnit.MM);
                timer.reset();
                phase = Phase.FORWARD;
            }
            return;
        }

        if (phase == Phase.FORWARD) {
            runSegmentUpdate("FORWARD", /*forward*/ +1.0, /*strafe*/ 0.0);
            double dxIn = mmToIn(odo.getPosX(DistanceUnit.MM) - startXmm);

            // Record max |vx| (axis max), not speed magnitude
            double vxIn = mmToIn(odo.getVelX(DistanceUnit.MM));
            maxAbsVxIn = Math.max(maxAbsVxIn, Math.abs(vxIn));

            if (Math.abs(dxIn) >= TEST_DISTANCE_IN || timer.seconds() >= TIMEOUT_SECONDS) {
                stopDrive();
                timer.reset();
                phase = Phase.PAUSE1;
            }
            return;
        }

        if (phase == Phase.PAUSE1) {
            stopDrive();
            telemetry.addLine("Pause between segments...");
            telemetry.addData("Time (s)", timer.seconds());
            telemetry.update();

            if (timer.seconds() >= PAUSE_SECONDS) {
                // begin strafe segment
                startXmm = odo.getPosX(DistanceUnit.MM);
                startYmm = odo.getPosY(DistanceUnit.MM);
                timer.reset();
                phase = Phase.STRAFE;
            }
            return;
        }

        if (phase == Phase.STRAFE) {
            // Strafe RIGHT: Pinpoint defines +Y as LEFT, so vy will usually be negative :contentReference[oaicite:13]{index=13}
            runSegmentUpdate("STRAFE RIGHT", /*forward*/ 0.0, /*strafe*/ +1.0);
            double dyIn = mmToIn(odo.getPosY(DistanceUnit.MM) - startYmm);

            double vyIn = mmToIn(odo.getVelY(DistanceUnit.MM));
            maxAbsVyIn = Math.max(maxAbsVyIn, Math.abs(vyIn));

            if (Math.abs(dyIn) >= TEST_DISTANCE_IN || timer.seconds() >= TIMEOUT_SECONDS) {
                stopDrive();
                phase = Phase.DONE;
            }
            return;
        }

        // DONE: show copy/paste forever
        if (phase == Phase.DONE) {
            stopDrive();
            telemetry.addLine("DONE ✅  Paste these into Constants.java");
            telemetry.addData("xVelocityRealValue (forward, in/s)", maxAbsVxIn);
            telemetry.addData("yVelocityRealValue (strafe,  in/s)", maxAbsVyIn);
            telemetry.addLine("");
            telemetry.addLine(String.format("public static double xVelocityRealValue = %.2f;", maxAbsVxIn));
            telemetry.addLine(String.format("public static double yVelocityRealValue = %.2f;", maxAbsVyIn));
            telemetry.update();
        }
    }

    @Override
    public void onStop() {
        stopDrive();
    }

    private void runSegmentUpdate(String label, double forward, double strafe) {
        double ramp = Math.min(1.0, timer.seconds() / RAMP_SECONDS);
        setDrivePowers(forward * MAX_POWER * ramp, strafe * MAX_POWER * ramp, 0.0);

        double vxIn = mmToIn(odo.getVelX(DistanceUnit.MM));
        double vyIn = mmToIn(odo.getVelY(DistanceUnit.MM));

        telemetry.addData("Segment", label);
        telemetry.addData("t (s)", timer.seconds());
        telemetry.addData("vx (in/s)", vxIn);
        telemetry.addData("vy (in/s)", vyIn);
        telemetry.addData("max |vx| (in/s)", maxAbsVxIn);
        telemetry.addData("max |vy| (in/s)", maxAbsVyIn);
        telemetry.update();
    }

    // Standard mecanum mixing (normalized)
    private void setDrivePowers(double forward, double strafe, double turn) {
        double flP = forward + strafe + turn;
        double frP = forward - strafe - turn;
        double blP = forward - strafe + turn;
        double brP = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(flP),
                Math.max(Math.abs(frP), Math.max(Math.abs(blP), Math.abs(brP)))));

        fl.setPower(flP / max);
        fr.setPower(frP / max);
        bl.setPower(blP / max);
        br.setPower(brP / max);
    }

    private void stopDrive() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
