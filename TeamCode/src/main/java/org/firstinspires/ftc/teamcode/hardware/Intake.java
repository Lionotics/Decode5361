package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

@Config
public class Intake extends Subsystem {


    // ---------------- FULL DETECTION TUNABLES (FTC Dashboard) ----------------
    /** If filtered velocity (ticks/sec) is below this for FULL_DEBOUNCE_MS while intaking => FULL. */
    public static double FULL_VELOCITY_TPS = 1900.0;

    /** How long velocity must stay below threshold to count as FULL (debounce). */
    public static long FULL_DEBOUNCE_MS = 100;

    /** Exponential moving average smoothing for velocity. 0.0-1.0 (higher = less smoothing). */
    public static double VEL_EMA_ALPHA = 0.20;

    /** If FULL and velocity rises above this, we clear FULL (prevents “sticky” full when balls leave). */
    public static double CLEAR_FULL_VELOCITY_TPS = 1950.0;

    /** Intake direction check: your eat() uses power -1, so “intaking” is power < -0.05. */
    private static final double INTAKING_POWER_EPS = 0.05;

    // ---------------- INTERNAL STATE ----------------
    private double velEmaTps = 0.0;
    private long belowThreshSinceMs = -1;
    private boolean full = false;

    public    boolean intaking = false;

    private  MotorEx intakeBack;
    private  MotorEx intakeFront;

    private Servo lightLeft;
    private  Servo lightRight;

    public  MotorGroup intake;

    public static  double RED   = 0.277;
    public static final double GREEN = 0.500;

    private Intake() { }

    public static Intake INSTANCE = new Intake();

    public void initialize() {


// If the motors are mirrored and spin opposite directions mechanically,
// reverse ONE of them (usually the right).
// private final MotorEx intakeRight = new MotorEx("IntakeRight").reversed();
         intakeBack = new MotorEx("IntakeBack");
         intakeFront = new MotorEx("IntakeFront");
         intakeFront.reverse();

          intake = new MotorGroup(intakeFront, intakeBack);

          intaking = false;

          lightRight = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "lightRight");
        lightLeft = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "lightLeft");

    }

    // ---------------- YOUR EXISTING COMMANDS ----------------
    public InstantCommand setPowerToIntake(double i) {
        return new InstantCommand(() -> intake.setPower(i));
    }

    public  void setLightColor(double colorPos) {
        lightRight.setPosition(colorPos);
        lightLeft.setPosition(colorPos);
    }

    @Override
    public void periodic() {
        if (full) {
            setLightColor(GREEN);
        }
    }

    public InstantCommand eat() {
        return new InstantCommand(() -> {
            if (intake.getPower() != 0) {
                intaking = false;
                intake.setPower(0);
            } else {
                intaking = true;
                intake.setPower(-1);
            }
        });
    }

    public InstantCommand spit() {
        return new InstantCommand(() -> {
            intaking = false;
            if (intake.getPower() != 0) {
                intake.setPower(0);
            } else {
                intake.setPower(1);
            }
            resetFullDetection();
        });
    }

    public Command loadBall(double loadDelaySecond) {
        return new SequentialGroup(
                new InstantCommand(() -> intake.setPower(-1)),
                new Delay(loadDelaySecond),
                new InstantCommand(() -> intake.setPower(0))
        );
    }

    // ---------------- FULL DETECTION API ----------------

    /** Call this once per loop (ex: from TeleOp.onUpdate()). */
    public void updateFullDetection() {

        // Only judge "fullness" while intaking. If not intaking, clear timers (and optionally full).
        if (!intaking) {
            belowThreshSinceMs = -1;
            return;
        }



        long nowMs = System.currentTimeMillis();

        // Raw velocity (ticks/sec). MotorEx wraps DcMotorEx, so Java access is typically getVelocity().
        double rawVelTps = Math.abs(intake.getVelocity());

        // EMA filter
        if (velEmaTps == 0.0) velEmaTps = rawVelTps;
        velEmaTps = VEL_EMA_ALPHA * rawVelTps + (1.0 - VEL_EMA_ALPHA) * velEmaTps;


        // Decide FULL based on sustained low velocity (struggling / stalling)
        if (velEmaTps < FULL_VELOCITY_TPS) {
            if (belowThreshSinceMs < 0) belowThreshSinceMs = nowMs;
            if (nowMs - belowThreshSinceMs >= FULL_DEBOUNCE_MS) {
                full = true;
            }
        } else {
            belowThreshSinceMs = -1;
            // If we were marked full, allow clearing when velocity recovers (balls moved out).
            if (full && velEmaTps > CLEAR_FULL_VELOCITY_TPS) {
                full = false;
            }
        }
    }

    public void resetFullDetection() {
        velEmaTps = 0.0;
        belowThreshSinceMs = -1;
        full = false;
    }

    public boolean isFull() {
        return full;
    }

    public double getRawVelocityTps() {
        return Math.abs(intake.getVelocity());
    }

    public double getFilteredVelocityTps() {
        return velEmaTps;
    }


}
