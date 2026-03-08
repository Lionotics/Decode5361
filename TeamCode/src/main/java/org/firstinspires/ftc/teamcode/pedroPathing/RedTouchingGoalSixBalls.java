package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.AutoScoreCommands;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Webcam;


@Config
@Autonomous(name = "RedTouchingGoalSixBallsAuto", group = "Autonomous")
public class RedTouchingGoalSixBalls extends AutoParent {

    // This auto's custom paths
    private PathChain pathToShootingInitial;

    private PathChain pathToWallBalls1;

    private PathChain pathToWallBalls2;

    private PathChain pathToShootingfromBalls;

    private PathChain pathToEnd;


    public static double angleToFaceGoal = 325;

    public static double angleToSuckInLineBalls = 180;

    public  static  double gettingLineBallX = 130;

    public  static  double gettingLineBallY = 87;


    public  static  double minIntakeMS = 700;

    public  static  double maxIntakeMS = 1600;

    public  static  double holdVelocityBefore = 1000;

    public  static  double desiredBearing = 0;


    private long intakeFullStartMs = -1; // when isFull first became true (continuous timer)


    @Override
    public  void onInit() {
        super.onInit();
        Webcam.INSTANCE.setSoleTagID(RED_TAG_ID); ;
    }


    @Override
    protected Pose getStartPose() {
        return new Pose(111, 135, Math.toRadians(270));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToShootingInitial = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(111, 135),
                        new Pose(90, 90)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(angleToFaceGoal))
                .build();

        pathToWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 90),
                        new Pose(90, gettingLineBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToSuckInLineBalls))
                .build();



        pathToWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, gettingLineBallY),
                        new Pose(gettingLineBallX, gettingLineBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();

        pathToShootingfromBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingLineBallX, gettingLineBallY),
                        new Pose(90, 90)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        pathToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 90),
                        new Pose(90, 130)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), 0)
                .build();
    }

    @Override
    protected int autonomousPathUpdate() {


        switch (pathState) {
            case 0:
                Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();

                follower.followPath(pathToShootingInitial);
                pathState = 1;
                break;

            case 1:
                // Wait until we are done driving to the shooting pose
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag() ) {
                    double angle = Webcam.INSTANCE.getBearing();
                    follower.turn(AngleUnit.normalizeRadians(Math.toRadians(angle-desiredBearing)));
                    pathState = 2;
                }
                break;


            case 2:
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag()) {
                    follower.breakFollowing();
                    if (scoreCmd == null) {
                        // Autonomous autoscore: NO faceBlueGoal
                        scoreCmd = AutoScoreCommands.autoAutoScoreNoFaceGoal();
                        scoreCmd.invoke();
                    } else if (scoreCmd.isDone() && Transfer.INSTANCE.scoreTimes >= 3) {
                        scoreCmd = null;
                        pathState = 3;  // continue your existing FSM
                    }
                }
                break;


            case 3:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToWallBalls1);
                    pathState = 4;
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathToWallBalls2);
                    pathState = 5;
                }
                break;

            case 5:
                Intake.INSTANCE.intake.setPower(0);
                Intake.INSTANCE.eat().invoke();
                turnStartMs = System.currentTimeMillis();
                intakeFullStartMs = -1;
                pathState = 6;

                break;

            case 6:
                long now = System.currentTimeMillis();

                // safety timeout since we entered intake state (set in case 5)
                long totalElapsed = now - turnStartMs;

                boolean full = Intake.INSTANCE.isFull();

                long fullFor = -1;

                if (full) {
                    // start the "continuous true" timer the moment it first becomes full
                    if (intakeFullStartMs < 0) intakeFullStartMs = now;

                    fullFor = now - intakeFullStartMs;

                } else {
                    // lost "full" -> reset continuous timer
                    intakeFullStartMs = -1;
                }

                // hard timeout fallback
                if (!follower.isBusy() || fullFor >= minIntakeMS || totalElapsed >= maxIntakeMS) {
                    Intake.INSTANCE.eat().invoke();
                    follower.breakFollowing();
                    follower.followPath(pathToShootingfromBalls);
                    Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();


                    intakeFullStartMs = -1;

                    pathState = 7;
                }
                break;



            case 7:
                // Wait until we are done driving to the shooting pose
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag() ) {
                    follower.breakFollowing();
                    double angle = Webcam.INSTANCE.getBearing();
                    follower.turn(AngleUnit.normalizeRadians(Math.toRadians(angle-desiredBearing)));
                    pathState = 8;
                }
                break;


            case 8:
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag()) {
                    follower.breakFollowing();
                    if (scoreCmd == null) {
                        // Autonomous autoscore: NO faceBlueGoal
                        scoreCmd = AutoScoreCommands.autoAutoScoreNoFaceGoal();
                        scoreCmd.invoke();
                    } else if (scoreCmd.isDone() && Transfer.INSTANCE.scoreTimes >= 3) {
                        scoreCmd = null;
                        pathState = 9;  // continue your existing FSM
                    }
                }
                break;



            case 9:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToEnd);
                    pathState = 10;
                }
                break;


        }

        return pathState;
    }

    @Override
    protected void addSubclassTelemetry() {
        // optional extra debugging
    }
}
