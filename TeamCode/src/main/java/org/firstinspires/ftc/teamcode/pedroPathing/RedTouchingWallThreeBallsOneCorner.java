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
import org.firstinspires.ftc.teamcode.hardware.Webcam;


@Config
@Autonomous(name = "RedTouchingWallThreeBallsOneCornerAuto", group = "Autonomous")
public class RedTouchingWallThreeBallsOneCorner extends AutoParent {

    // This auto's custom paths
    private PathChain pathToShootingInitial;

    private PathChain pathToFirstWallBalls1;

    private PathChain pathToFirstWallBalls2;

    private PathChain pathToSecondWallBalls1;

    private PathChain pathToSecondWallBalls2;


    private PathChain pathToThirdWallBalls1;
    private PathChain pathToThirdWallBalls2;


    private PathChain pathToShootingfromFirstBalls;

    private PathChain pathToShootingfromSecondBalls;

    private PathChain pathToShootingfromThirdBalls;


    private PathChain pathToEnd1;

    private PathChain pathToEnd2;


    public static double angleToFaceGoal = 340;

    public static double angleToSuckInLineBalls = 180;

    public  static  double gettingFirstWallBallX = 134;

    public  static  double gettingFirstWallBallY = 10;

    public  static  double gettingSecondWallBallX = 134;

    public  static  double gettingSecondWallBallY = 8;

    public  static  double gettingThirdWallBallX = 134;

    public  static  double gettingThirdWallBallY = 10;




    public  static  double minIntakeMS = 2000;

    public  static  double maxIntakeMS = 3000;

    public  static  double holdVelocityBefore = 1600;

    public  static  double desiredBearing = 2;


    private long intakeFullStartMs = -1; // when isFull first became true (continuous timer)


    @Override
    public  void onInit() {
        super.onInit();
         Webcam.INSTANCE.setSoleTagID(RED_TAG_ID); ;
    }


    @Override
    protected Pose getStartPose() {
        return new Pose(88, 8, Math.toRadians(90));
    }

    @Override
    protected void buildPaths() {
        // Your existing Path1/Path2 logic moved here
        pathToShootingInitial = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000),
                        new Pose(88, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(angleToFaceGoal))
                .build();

        pathToFirstWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, 15),
                        new Pose(88, gettingFirstWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();



        pathToFirstWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, gettingFirstWallBallY),
                        new Pose(gettingFirstWallBallX, gettingFirstWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();

        pathToSecondWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, 15),
                        new Pose(88, gettingSecondWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToSecondWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, gettingSecondWallBallY),
                        new Pose(gettingSecondWallBallX, gettingSecondWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToThirdWallBalls1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, 15),
                        new Pose(88, gettingThirdWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();

        pathToThirdWallBalls2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88, gettingThirdWallBallY),
                        new Pose(gettingThirdWallBallX, gettingThirdWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToSuckInLineBalls))
                .build();


        pathToShootingfromFirstBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingFirstWallBallX, gettingFirstWallBallY),
                        new Pose(88, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        pathToShootingfromSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingSecondWallBallX, gettingSecondWallBallY),
                        new Pose(88, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();

        pathToShootingfromThirdBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingThirdWallBallX, gettingThirdWallBallY),
                        new Pose(88, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build();


        /*pathToShootingfromSecondBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingSecondWallBallX, gettingSecondWallBallY),
                        new Pose(88, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), Math.toRadians(angleToFaceGoal))
                .build(); */


        pathToEnd1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(gettingFirstWallBallX , gettingFirstWallBallY),
                        new Pose(88, gettingFirstWallBallY)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(angleToSuckInLineBalls), 0)
                .build();


        pathToEnd2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(88 , gettingFirstWallBallY),
                        new Pose(88, 35)
                ))
                .setLinearHeadingInterpolation(0, 0)
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
                    } else if (scoreCmd.isDone() ) {
                        scoreCmd = null;
                        pathState = 3;  // continue your existing FSM
                    }
                }
                break;






            case 3:
                if ( !follower.isBusy() ) {
                    follower.breakFollowing();
                    follower.followPath(pathToFirstWallBalls1);
                    pathState = 4;
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(pathToFirstWallBalls2);
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
                    follower.followPath(pathToEnd1);


                    intakeFullStartMs = -1;

                    pathState = 7;
                }
                break;



            case 7:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    follower.followPath(pathToEnd2);
                    pathState = 8;
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
