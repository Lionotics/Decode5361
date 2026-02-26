package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.AutoScoreCommands;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Webcam;


@Config
@Autonomous(name = "RedTouchingWallAuto", group = "Autonomous")
public class RedTouchingWall extends AutoParent {

    // This auto's custom paths
    private PathChain pathToShootingInitial;
    private PathChain pathToBalls;

    public static double angleToFaceGoal = 335;

    public  static  double minIntakeMS = 100;

    public  static  double maxIntakeMS = 3000;


    @Override
    public  void onInit() {
        super.onInit();
        DriveTrain.INSTANCE.setGoalID(24);
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
                        new Pose(85, 15)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(angleToFaceGoal))
                .build();

        pathToBalls = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(85, 15),
                        new Pose(132, 8)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    @Override
    protected int autonomousPathUpdate() {


        switch (pathState) {
            case 0:
                follower.followPath(pathToShootingInitial);
                pathState = 1;
                break;

            case 1:
                // Wait until we are done driving to the shooting pose
                if (!follower.isBusy() && Webcam.INSTANCE.seesTag() ) {
                    double angle = Webcam.INSTANCE.getBearing();
                    follower.turn(AngleUnit.normalizeRadians(Math.toRadians(angle)));
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


            case  3:
                    follower.turnTo(180);
                    pathState = 4;
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.breakFollowing();
                    follower.followPath(pathToBalls);
                    pathState = 5;
                }
                break;

                case 5:
                    Intake.INSTANCE.intake.setPower(0);
                    Intake.INSTANCE.eat().invoke();
                    turnStartMs = System.currentTimeMillis();
                    pathState = 6;

                break;

            case 6:
                double timeElapsed = System.currentTimeMillis() - turnStartMs;
                if ( (timeElapsed >= minIntakeMS && Intake.INSTANCE.isFull()) || (timeElapsed >= maxIntakeMS) ) {
                    Intake.INSTANCE.eat();
                    follower.breakFollowing();
                    follower.followPath(pathToBalls);
                    pathState = 7;
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
