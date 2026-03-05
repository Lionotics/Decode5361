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
@Autonomous(name = "BlueTouchingWallNineBallsHigherBallFirstAuto", group = "Autonomous")
public class BlueTouchingWallNineBallsLineHigherBallFirst extends AutoParent {

    // This auto's custom paths
    private PathChain pathToShootingInitial;

    private PathChain pathToLowerWallBalls1;

    private PathChain pathToLowerWallBalls2;

    private PathChain pathToHigherWallBalls1;

    private PathChain pathToHigherWallBalls2;

    private PathChain pathToShootingfromLowerBalls;

    private PathChain pathToShootingfromHigherBalls;


    private PathChain pathToShootingfromHigherBallsAvoidingOtherBalls1;

    private PathChain pathToShootingfromHigherBallsAvoidingOtherBalls2;


    private PathChain pathToEnd;



    public  static  double desiredBearing = -4;

    public static double angleToFaceGoal = 25;

    public static double angleToSuckInLineBalls = 0;

    public  static  double gettingLowerWallBallX = 6;

    public  static  double gettingLowerWallBallY = 33;


    public  static  double gettingHigherWallBallX = 6;

    public  static  double gettingHigherWallBallY = 56;


    public  static  double minIntakeMS = 800;

    public  static  double maxIntakeMS = 3000;

    public  static  double holdVelocityBefore = 1550;

    public  static  boolean shouldHoldVelocityBefore = true;


    private long intakeFullStartMs = -1; // when isFull first became true (continuous timer)


    @Override
    public  void onInit() {
        super.onInit();
        Webcam.INSTANCE.setSoleTagID(BLUE_TAG_ID); ;
    }


    @Override
    protected Pose getStartPose() {
        return new Pose(56, 8, Math.toRadians(90));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToShootingInitial = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 8.000),
                        new Pose(56, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(angleToFaceGoal))
                .build();

        pathToLowerWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 15),
                        new Pose(56, gettingLowerWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();



        pathToLowerWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, gettingLowerWallBallY),
                        new Pose(gettingLowerWallBallX, gettingLowerWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToHigherWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 15),
                        new Pose(56, gettingHigherWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToHigherWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, gettingHigherWallBallY),
                        new Pose(gettingHigherWallBallX, gettingHigherWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();

        pathToShootingfromLowerBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingLowerWallBallX, gettingLowerWallBallY),
                        new Pose(56, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();

        pathToShootingfromHigherBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingHigherWallBallX, gettingHigherWallBallY),
                        new Pose(56, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        pathToShootingfromHigherBallsAvoidingOtherBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingHigherWallBallX, gettingHigherWallBallY),
                        new Pose(56, gettingHigherWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToShootingfromHigherBallsAvoidingOtherBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, gettingHigherWallBallY),
                        new Pose(56, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        pathToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 15),
                        new Pose(56, 35)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(180))
                .build();
    }

    @Override
    protected int autonomousPathUpdate() {


        switch (pathState) {
            case 0:
                if (shouldHoldVelocityBefore) {
                    Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();
                }
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
                    } else if (Transfer.INSTANCE.scoreTimes >= 3 ) {
                        scoreCmd = null;
                        pathState = 9;  // continue your existing FSM
                    }
                }
                break;






            case 3:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToLowerWallBalls1);
                    pathState = 4;
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathToLowerWallBalls2);
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
                  if (shouldHoldVelocityBefore) {
                      Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();
                  }
                    Intake.INSTANCE.eat().invoke();
                    follower.breakFollowing();
                    follower.followPath(pathToShootingfromLowerBalls);

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
                    } else if (Transfer.INSTANCE.scoreTimes >= 3) {
                        scoreCmd = null;
                        pathState = 15;  // continue your existing FSM
                    }
                }
                break;


            case 9:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToHigherWallBalls1);
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(pathToHigherWallBalls2);
                    pathState = 11;
                }
                break;

            case 11:
                Intake.INSTANCE.intake.setPower(0);
                Intake.INSTANCE.eat().invoke();
                turnStartMs = System.currentTimeMillis();
                intakeFullStartMs = -1;
                pathState = 12;

                break;

            case 12:
                now = System.currentTimeMillis();

                // safety timeout since we entered intake state (set in case 5)
                totalElapsed = now - turnStartMs;

                full = Intake.INSTANCE.isFull();

                fullFor = -1;

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
                    follower.followPath(pathToShootingfromHigherBallsAvoidingOtherBalls1);
                    Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();


                    intakeFullStartMs = -1;

                    pathState = 1205;
                }
                break;


            case 1205:
                if (!follower.isBusy()) {
                    follower.followPath(pathToShootingfromHigherBallsAvoidingOtherBalls2);
                    pathState = 13;
                }

                break;



            case 13:
                // Wait until we are done driving to the shooting pose
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag() ) {
                    follower.breakFollowing();
                    double angle = Webcam.INSTANCE.getBearing();
                    follower.turn(AngleUnit.normalizeRadians(Math.toRadians(angle-desiredBearing)));
                    pathState = 14;
                }
                break;


            case 14:
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag()) {
                    follower.breakFollowing();
                    if (scoreCmd == null) {
                        // Autonomous autoscore: NO faceBlueGoal
                        scoreCmd = AutoScoreCommands.autoAutoScoreNoFaceGoal();
                        scoreCmd.invoke();
                    } else if (Transfer.INSTANCE.scoreTimes >= 3 ) {
                        scoreCmd = null;
                        pathState = 3;  // continue your existing FSM
                    }
                }
                break;


            case 15:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToEnd);
                    pathState = 16;
                }
                break;


            case 16:
                if (!follower.isBusy()) {
                    follower.turnTo(Math.toRadians(180));
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
