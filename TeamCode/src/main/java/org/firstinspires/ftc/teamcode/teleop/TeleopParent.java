package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.CommandManager;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.AutoScoreCommands;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.OuttakeRotator;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Webcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class TeleopParent extends NextFTCOpMode {

    public final int BLUE_TAG_ID = 20;
    public final int RED_TAG_ID = 24;

    public static double targetHoodPosition = 0.05;

    public  static  double testTiltAngle = 0;

    private Follower follower;
    public Command driverControlled;

    private Command teleopAutoScoreCmd = null;


    public TeleopParent() {
        super(DriveTrain.INSTANCE,Intake.INSTANCE, Transfer.INSTANCE,Outtake.INSTANCE, OuttakeRotator.INSTANCE, Webcam.INSTANCE);

    }

    @Override
    public void onStartButtonPressed() {
        if (Webcam.ftcDashBoardTurnedOn) {
            FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);
        }


        follower = Constants.createFollower(hardwareMap);

        // Optional but recommended: set a starting pose (pick something reasonable)
        follower.setStartingPose(new Pose(0, 0, 0));

        GamepadEx gp1 = gamepadManager.getGamepad1();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverControlled = DriveTrain.INSTANCE.Drive(gp1, false);
        driverControlled.invoke();

       gp1.getA().setPressedCommand(() -> Test());

        gp1.getB().setPressedCommand(() -> Intake.INSTANCE.eat());


        gp1.getDpadDown().setPressedCommand(() -> Outtake.INSTANCE.handleMotor(0));

        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.spit());




        gp1.getDpadUp().setHeldCommand( ()-> Transfer.INSTANCE.kickBall());



        gp1.getRightBumper().setPressedCommand(() -> {
            teleopAutoScoreCmd = AutoScoreCommands.teleopAutoScore(
                    follower,
                    driverControlled,
                    testTiltAngle,
                    true
            );

            return teleopAutoScoreCmd;
        });



        gp1.getLeftBumper().setPressedCommand(() -> {
            if (follower.isBusy()) return new NullCommand();
            return new SequentialGroup(
                   new InstantCommand(() -> driverControlled.stop(true)),
                    DriveTrain.INSTANCE.faceGoal(follower, testTiltAngle),
                    new InstantCommand(() -> driverControlled.invoke())
            );
        });



// Return to normal field-centric driving
        gp1.getDpadLeft().setPressedCommand(() ->
                new InstantCommand(   ()-> {
                    if (teleopAutoScoreCmd != null && !teleopAutoScoreCmd.isDone()) {
                        CommandManager.INSTANCE.cancelCommand(teleopAutoScoreCmd);

                        teleopAutoScoreCmd = null;
                    }

                    follower.breakFollowing();
                    driverControlled.invoke();

                }
                )
        );

    }

    @Override
    public void onUpdate() {

        if (DriveTrain.INSTANCE.odometry != null) {

            Pose2D pose = DriveTrain.INSTANCE.odometry.getPosition();

            double xInches = pose.getX(DistanceUnit.INCH);
            double yInches = pose.getY(DistanceUnit.INCH);
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);

            telemetry.addData("Odo X (in)", xInches);
            telemetry.addData("Odo Y (in)", yInches);

            double h360 = (headingDeg % 360 + 360) % 360;
            telemetry.addData("Odo Heading (deg)", h360);
        }

        telemetry.addData("IMU Heading (deg)", DriveTrain.INSTANCE.getIMUHeading());



        telemetry.addData("Hood Position", OuttakeRotator.INSTANCE.getHoodPosition());


        telemetry.addData("INTAKE FULL (3 balls)", Intake.INSTANCE.isFull() ? "YES" : "NO");




        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);


        telemetry.addData("lastFaceGoalAngleRelative", DriveTrain.lastFaceGoalAngleRelative);
        telemetry.addData("lastFaceGoalAngleAbsolute", DriveTrain.lastFaceGoalAngleAbsolute);

        telemetry.addData("Blue Tag X Estimate (in)", DriveTrain.blueTagX_in);
        telemetry.addData("Blue Tag Y Estimate (in)", DriveTrain.blueTagY_in);




        Webcam.INSTANCE.addTelemetry(telemetry);




        telemetry.update();
    }

    @Override
    public  void onStop() {
        Webcam.INSTANCE.close();
        if (Webcam.ftcDashBoardTurnedOn) {
            FtcDashboard.getInstance().stopCameraStream();
        }
    }

    public Command Test(){
            return new Command() {
                    Command inrCmd;
                    @Override
                    public void start () {
                        Outtake.motorVelocityTarget = 1650;
                        inrCmd =
                        new ParallelGroup(
                                Outtake.INSTANCE.holdVelocity(Outtake.motorVelocityTarget)//,
                               // OuttakeRotator.INSTANCE.setHoodPosition(targetHoodPosition)
                        );
                        inrCmd.invoke();
                    }
                    @Override
                    public void update () {

                    }

                    @Override
                    public boolean isDone () {
                        return (inrCmd != null && inrCmd.isDone());
                    }

        };
    }


}
