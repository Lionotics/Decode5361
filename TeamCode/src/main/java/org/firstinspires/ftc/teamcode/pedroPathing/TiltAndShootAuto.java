package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

// Your Pedro constants factory
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

/**
 * Auto that:
 *  1. Starts parallel to the wall (heading 0)
 *  2. Tilts TILT_DEG degrees
 *  3. Waits for shooting
 *  4. Tilts back to heading 0 (parallel to wall again)
 */

@Config
@Autonomous(name = "5361auto", group = "Auto")
public class TiltAndShootAuto extends LinearOpMode {

    // How far you want to tilt (counterclockwise positive)
    public static final double TILT_DEG = 25.0;
    public static final double TILT_RAD = Math.toRadians(TILT_DEG);

    // Starting pose: parallel to wall, facing "upfield" (heading 0)
    // You can tweak x/y to match the actual alliance start location.
    private final Pose startPose = new Pose(9.0, 60.0, 0.0);
    private final Pose tiltPose  = new Pose(9.0, 60.0, TILT_RAD);

    private Follower follower;

    private PathChain tiltPath;
    private PathChain returnPath;

    public static double motorDelaySeconds = 0.5;


    /**
     * Build the two simple paths:
     *  - tiltPath:  startPose -> tiltPose (rotate + tiny move)
     *  - returnPath: tiltPose -> startPose (rotate back)
     *
     * For Pedro, we give it a small translational difference as well,
     * but the main thing we care about is heading interpolation.
     */
    private void buildPaths() {
        // Slightly nudge the x so Pedro has a real path, not literally 0-length
        Pose tiltPoseWithNudge = new Pose(
                tiltPose.getX() + 1.0,  // 1 inch forward just to satisfy path math
                tiltPose.getY(),
                tiltPose.getHeading()
        );

        tiltPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, tiltPoseWithNudge))
                // Turn from 0 to TILT_RAD over the path
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        tiltPose.getHeading(),
                        1.0
                )
                .build();

        returnPath = follower.pathBuilder()
                .addPath(new BezierLine(tiltPoseWithNudge, startPose))
                // Turn back from TILT_RAD to 0 over the path
                .setLinearHeadingInterpolation(
                        tiltPose.getHeading(),
                        startPose.getHeading(),
                        1.0
                )
                .build();
    }

    @Override
    public void runOpMode() {
        Outtake.INSTANCE.initialize();
        Intake.INSTANCE.initialize();
        Transfer.INSTANCE.initialize();

        // Create Pedro follower using your Constants class (Pedro 2.x style)
        follower = Constants.createFollower(hardwareMap);

        // Make sure Pedro starts with the same pose we will simulate in MeepMeep
        follower.setStartingPose(startPose);

        // Build our paths
        buildPaths();

        telemetry.addData("Status", "TiltAndShootAuto init complete");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // 1. Tilt to shooting angle
        runPath(tiltPath);


        Outtake.INSTANCE.handleMotor( Outtake.motorVelocityClose );
        new Delay(motorDelaySeconds);
        // 2. Shoot (replace with your shooter logic)
        for (int i = 0; i < 3; i++) {
             Transfer.INSTANCE.kickBall();
        }

        // 3. Tilt back to 0°
        runPath(returnPath);
    }

    /**
     * Helper to follow a path and keep Pedro updated until it finishes.
     */
    private void runPath(PathChain path) {
        follower.followPath(path);  // tell Pedro to start the path:contentReference[oaicite:4]{index=4}

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();      // Pedro's main loop update:contentReference[oaicite:5]{index=5}

            Pose pose = follower.getPose();
            telemetry.addData("x", pose.getX());
            telemetry.addData("y", pose.getY());
            telemetry.addData("heading (deg)", Math.toDegrees(pose.getHeading()));
            telemetry.update();

            idle();
        }
    }
}
