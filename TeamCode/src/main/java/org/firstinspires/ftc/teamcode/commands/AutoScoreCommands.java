package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.OuttakeRotator;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.Set;

/**
 * AutoScoreCommands
 * - TeleOp version: optionally faces the goal and stops/restores driver-controlled command.
 * - Auto version: does NOT call faceBlueGoal (assumes your path already points you correctly).
 */
public final class AutoScoreCommands {

    private AutoScoreCommands() {}


    public static class HoldPoseWhileActive extends Command {
        private final Follower follower;

        private  Command driverControlledArg;

        private Pose holdPose;

        public HoldPoseWhileActive(Follower follower, Command b) {
            this.follower = follower;
            this.driverControlledArg = b;
        }

        @Override
        public void start() {
            // Hold the current pose at the moment shooting starts
            holdPose = follower.getPose();
            follower.holdPoint(holdPose);
        }

        @Override
        public void update() {
            // This is the critical part: keep Pedro “alive” so it actually resists pushes
            follower.update();

            // Optional: if you want it to “re-lock” to the original pose even harder:
            // follower.holdPoint(holdPose);
        }

        @Override
        public boolean isDone() {
            return Transfer.INSTANCE.scoreTimes >= 2 && Transfer.INSTANCE.speederUpper; // runs until canceled by the group finishing
        }

        @Override
        public void stop(boolean interrupted) {
            // Release Pedro control cleanly
            follower.breakFollowing();
            driverControlledArg.invoke();
        }

        @Override
        @NotNull
        public Set<Subsystem> getSubsystems() {
            return Collections.singleton(DriveTrain.INSTANCE);
        }
    }


    /**
     * TeleOp AutoScore: stops driver control, faces goal (optional), waits for tag, shoots 3.
     * Mirrors your old TeleopParent.autoScore() behavior, but now reusable.
     */
    public static Command teleopAutoScore(
            Follower follower,
            Command driverControlled,
            double tiltAngle,
            boolean doFaceGoal
    ) {
        return new SequentialGroup(

                new InstantCommand(() -> driverControlled.stop(true)),

                new InstantCommand(Webcam::beginCameraUse),

                // optional face step (your old behavior did faceBlueGoal here)
                DriveTrain.INSTANCE.faceGoal(follower, tiltAngle),


                  new ForcedParallelCommand(
                       new HoldPoseWhileActive(follower,driverControlled)
                ),


                // old teleop behavior waited until tag is seen
                new WaitUntil(() -> Webcam.INSTANCE.seesTag()),



                buildShootFromDistanceCommand(Transfer.kickDelaySecondsTeleop, Transfer.preReturnDelaySecondsTeleop),

        new InstantCommand(Webcam::endCameraUse),


                Outtake.INSTANCE.stopMotor()


                );
    }

    /**
     * Autonomous AutoScore: NO faceBlueGoal.
     * Uses webcam range if tag is visible; otherwise uses DriveTrain tag estimate + odometry;
     * otherwise falls back to 30 inches.
     *
     * NOTE: It does not wait for a tag by default. If you want to wait for tag visibility,
     * do it in the auto state machine before invoking this command (or wrap it with WaitUntil).
     */
    public static Command autoAutoScoreNoFaceGoal() {
        return  new SequentialGroup(
                buildShootFromDistanceCommand(Transfer.kickDelaySecondsAuto, Transfer.preReturnDelaySecondsAuto),
                new WaitUntil( ()-> Transfer.INSTANCE.scoreTimes == 3),
                new InstantCommand( ()-> Outtake.INSTANCE.stopMotor().invoke() )
         );
    }

    /**
     * Shared “compute distance and run outtake/hood/3 shots/stop” logic.
     * This is the core of what your old TeleopParent.autoScore() did after faceBlueGoal.
     */
    private static Command buildShootFromDistanceCommand(double kickDelayArg, double preReturnArg) {
        return new Command() {

            private boolean startedInner = false;
            private Command inner = null;


            @Override
            public void start() {
                Webcam.beginCameraUse();

                        startedInner = false;
                inner = null;
                Transfer.INSTANCE.scoreTimes = 0;
            }

            @Override
            public void update() {
                // Don't even build the shooting routine until the tag is visible.
                if (!startedInner) {
                    if (!Webcam.INSTANCE.seesTag()) {
                        return; // keep waiting
                    }

                    // Tag is visible NOW -> we can compute distance and begin shooting.
                    startedInner = true;

                    Transfer.INSTANCE.scoreTimes = 0;

                    double distIn = computeDistanceFromGoalInches(); // guaranteed tag-based
                    double targetVel = Outtake.INSTANCE.distanceToVelocity(distIn);

                    Transfer.INSTANCE.speederUpper = false;

                    inner = new SequentialGroup(
                            new ForcedParallelCommand(Outtake.INSTANCE.holdVelocity(targetVel)),
                            OuttakeRotator.INSTANCE.setHoodPosition(
                                    Outtake.INSTANCE.distanceToHoodPosition(distIn)
                            ),
                            score3Times(kickDelayArg, preReturnArg)
                    );


                    inner.invoke();
                    return;
                }

                // Once inner exists, it will progress through the command scheduler.
                // No work needed here unless you want telemetry.
            }

            @Override
            public boolean isDone() {
                // We're done only after we have started and the inner routine is finished.
                return startedInner && inner != null && inner.isDone();
            }

            @Override
            public void stop(boolean interrupted) {
                Webcam.endCameraUse();

                if (interrupted) {
                    Outtake.INSTANCE.stopMotor().invoke();
                }
            }
        };
    }

    /**
     * Tag-required distance.
     * Since buildShootFromDistanceCommand() waits until seesTag(), this will be safe.
     */
    private static double computeDistanceFromGoalInches() {
        if (!Webcam.INSTANCE.seesTag()) {
            // Defensive: should never happen because the caller waits, but prevents accidental misuse.
            throw new IllegalStateException("computeDistanceFromGoalInches() called without a visible tag.");
        }
        return Webcam.INSTANCE.getRange();
    }

    /**
     * Moved from TeleopParent.score3Times().
     */
    public static Command score3Times(double kickDelayArg, double preReturnSecondArg) {
        return new Command() {
            private Command currentShot;
            private boolean shotYet = false;

            @Override
            public void start() {
                Transfer.INSTANCE.scoreTimes = 0;
                Transfer.INSTANCE.speederUpper = false;
            }

            @Override
            public void update() {
                if (Transfer.INSTANCE.scoreTimes >= 3) return;

                if (!shotYet || (currentShot != null && currentShot.isDone())) {
                    shotYet = true;
                    currentShot = Transfer.INSTANCE.kickBall(kickDelayArg,preReturnSecondArg);
                    currentShot.invoke();
                }
            }

            @Override
            public boolean isDone() {
                return (shotYet && Transfer.INSTANCE.scoreTimes >= 3 && currentShot.isDone());
            }
        };
    }
}