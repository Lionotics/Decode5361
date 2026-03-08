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
@Autonomous(name = "RedTouchingGoalNineBallsAuto", group = "Autonomous")
public class RedTouchingGoalNineBalls extends AutoParent {

    // This auto's custom paths
    private PathChain pathToShootingInitial;

    private PathChain pathToFirstBalls1;

    private PathChain pathToFirstBalls2;


    private PathChain pathToSecondBalls1;

    private PathChain pathToSecondBalls2;

    private PathChain pathToShootingFromFirstBalls;

    private PathChain pathToShootingFromSecondBalls1;

    private PathChain pathToShootingFromSecondBalls2;



    private PathChain pathToGate1;

    private PathChain pathToGate2;

    private PathChain pathToEnd;


    public static double angleToFaceGoal = 315;

    public static double angleToSuckInLineBalls = 180;

    public  static  double gettingFirstBallX = 130;

    public  static  double gettingFirstBallY = 89;


    public  static  double gettingSecondBallX = 130;

    public  static  double gettingSecondBallY = 65;


    public  static  double GateY =  80;

    public  static  double preGateX = 100;

    public  static  double GateX = 129;


    public  static  double minIntakeMS = 800;

    public  static  double maxIntakeMS = 1600;

    public  static  double holdGoldForMS = 2000;

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

        pathToFirstBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 90),
                        new Pose(90, gettingFirstBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToSuckInLineBalls))
                .build();



        pathToFirstBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, gettingFirstBallY),
                        new Pose(gettingFirstBallX, gettingFirstBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToSecondBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, 90),
                        new Pose(90, gettingSecondBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToSecondBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, gettingSecondBallY),
                        new Pose(gettingSecondBallX, gettingSecondBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();



        pathToShootingFromFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(GateX, GateY),
                        new Pose(90, 90)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        pathToShootingFromSecondBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingSecondBallX, gettingSecondBallY),
                        new Pose(90, gettingSecondBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();

        pathToShootingFromSecondBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(90, gettingSecondBallY),
                        new Pose(90, 90)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToFaceGoal), Math.toRadians(angleToFaceGoal))
                .build();



        pathToGate1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingFirstBallX, gettingFirstBallY),
                        new Pose(preGateX, GateY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(270))
                .build();


        pathToGate2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(preGateX, GateY),
                        new Pose(GateX, GateY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
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
                    follower.followPath(pathToFirstBalls1);
                    pathState = 4;
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathToFirstBalls2);
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
                    follower.followPath(pathToGate1);
                    Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();


                    intakeFullStartMs = -1;

                    pathState = 6005;
                }
                break;


            case 6005:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToGate2);
                    turnStartMs = System.currentTimeMillis();
                    pathState = 6010;
                }
                break;


            case 6010:
                now = System.currentTimeMillis();
                totalElapsed = now - turnStartMs;


                if (totalElapsed >= holdGoldForMS  ) {
                    follower.breakFollowing();
                    follower.followPath(pathToShootingFromFirstBalls);
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
                    follower.followPath(pathToSecondBalls1);
                    pathState = 10;
                }
                break;


            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(pathToSecondBalls2);
                    turnStartMs = System.currentTimeMillis();
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
                    follower.followPath(pathToShootingFromSecondBalls1);
                    Outtake.INSTANCE.holdVelocity(holdVelocityBefore).invoke();


                    intakeFullStartMs = -1;

                    pathState = 12005;
                }
                break;


            case 12005:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToShootingFromSecondBalls2);
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
                    } else if (scoreCmd.isDone() && Transfer.INSTANCE.scoreTimes >= 3) {
                        scoreCmd = null;
                        pathState = 15;  // continue your existing FSM
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


        }

        return pathState;
    }

    @Override
    protected void addSubclassTelemetry() {
        // optional extra debugging
    }
}
