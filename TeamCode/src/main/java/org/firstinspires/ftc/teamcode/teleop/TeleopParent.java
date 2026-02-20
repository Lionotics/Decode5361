package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    public  static  double testTiltAngle = 90;

    private Follower follower;
    public Command driverControlled;

    public TeleopParent() {
        super(DriveTrain.INSTANCE,Intake.INSTANCE, Transfer.INSTANCE,Outtake.INSTANCE, OuttakeRotator.INSTANCE, Webcam.INSTANCE);

    }


    @Override
    public void onStartButtonPressed() {
       FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);

        follower = Constants.createFollower(hardwareMap);

        // Optional but recommended: set a starting pose (pick something reasonable)
        follower.setStartingPose(new Pose(0, 0, 0));

        GamepadEx gp1 = gamepadManager.getGamepad1();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverControlled = DriveTrain.INSTANCE.Drive(gp1, false);
        driverControlled.invoke();

      //  gp1.getA().setPressedCommand(() -> Test());

        gp1.getB().setPressedCommand(() -> Intake.INSTANCE.eat());


        gp1.getDpadDown().setPressedCommand(() -> Outtake.INSTANCE.handleMotor(0));

        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.spit());




        gp1.getDpadUp().setHeldCommand( ()-> Transfer.INSTANCE.kickBall());



        gp1.getRightBumper().setPressedCommand( ()-> autoScore() );



        gp1.getLeftBumper().setPressedCommand(() ->
                new SequentialGroup(
                        new InstantCommand(() -> driverControlled.stop(true)),
                        DriveTrain.INSTANCE.faceBlueGoal(follower, testTiltAngle),
                        new InstantCommand(() -> driverControlled.invoke())
                )
        );



// Return to normal field-centric driving
        gp1.getDpadLeft().setPressedCommand(() ->
                new InstantCommand(   ()-> {
                    driverControlled.invoke();
                }
                )
        );




    }

    @Override
    public void onUpdate() {

        if (DriveTrain.INSTANCE.odometry != null) {
            DriveTrain.INSTANCE.odometry.update(); // read sensors and update internal pose

            Pose2D pose = DriveTrain.INSTANCE.odometry.getPosition();

            double xInches = pose.getX(DistanceUnit.INCH);
            double yInches = pose.getY(DistanceUnit.INCH);
            double headingDeg = pose.getHeading(AngleUnit.DEGREES);

            telemetry.addData("Odo X (in)", xInches);
            telemetry.addData("Odo Y (in)", yInches);
            telemetry.addData("Odo Heading (deg)", headingDeg);
        }

        telemetry.addData("Hood Position", OuttakeRotator.INSTANCE.getHoodPosition());
        telemetry.addData("Hood Position Target", targetHoodPosition);

        // Update intake fullness detection once per loop
        Intake.INSTANCE.updateFullDetection();

        telemetry.addData("INTAKE FULL (3 balls)", Intake.INSTANCE.isFull() ? "YES" : "NO");




        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);

        telemetry.addData("lastFaceBlueGoalAngle", DriveTrain.lastFaceBlueGoalAngle);

        Webcam.INSTANCE.addTelemetry(telemetry);




        telemetry.update();
    }

    @Override
    public  void onStop() {
        Webcam.INSTANCE.close();
       FtcDashboard.getInstance().stopCameraStream();
    }



    public Command autoScore() {
        return new SequentialGroup(
                DriveTrain.INSTANCE.faceBlueGoal(follower,testTiltAngle),

                // This step runs ONLY after faceBlueGoal is finished
                new Command() {
                    private Command afterFace;

                    @Override
                    public void start() {
                        double webCamDistance;
                        if (Webcam.INSTANCE.seesTag()) {
                            webCamDistance = Webcam.INSTANCE.getRange();
                        } else if (DriveTrain.haveTagEstimate && DriveTrain.INSTANCE.odometry != null) {
                            // Make sure pose is fresh right now (not just onUpdate)
                            DriveTrain.INSTANCE.odometry.update();

                            Pose2D pose = DriveTrain.INSTANCE.odometry.getPosition();
                            double rx = pose.getX(DistanceUnit.INCH);
                            double ry = pose.getY(DistanceUnit.INCH);

                            double dx = DriveTrain.blueTagX_in - rx;
                            double dy = DriveTrain.blueTagY_in - ry;

                            webCamDistance = Math.hypot(dx, dy); // inches
                        } else {
                            webCamDistance = 30.0; // last-resort fallback if you *never* saw the tag yet
                        }

                        double targetTempRaw = Outtake.INSTANCE.distanceToVelocity(webCamDistance);

                        afterFace = new SequentialGroup(
                                new ForcedParallelCommand(Outtake.INSTANCE.holdVelocity(targetTempRaw)),
                                OuttakeRotator.INSTANCE.setHoodPosition(
                                        Outtake.INSTANCE.distanceToHoodPosition(webCamDistance)
                                ),
                                score3Times(),
                                Outtake.INSTANCE.stopMotor()
                        );

                        afterFace.invoke();
                    }

                    @Override
                    public void update() { }

                    @Override
                    public boolean isDone() {
                        return afterFace != null && afterFace.isDone();
                    }

                    @Override
                    public void stop(boolean interrupted) {
                        if (interrupted) {
                            Outtake.INSTANCE.stopMotor().invoke();
                        }
                    }
                },
                new InstantCommand(() -> driverControlled.invoke())
        );
    }





    public Command score3Times(){
       /*  return new SequentialGroup(
                Transfer.INSTANCE.kickBall(),
                Transfer.INSTANCE.kickBall(),
                Transfer.INSTANCE.kickBall()
        );  */

       return new Command() {
            private Command currentShot;


            boolean shotYet = false;

            @Override
            public void start() {
                Transfer.INSTANCE.scoreTimes = 0;

               // currentShot = Transfer.INSTANCE.kickBall();
               // currentShot.invoke(); // schedule first shot ONCE
            }

            @Override
            public void update() {
                if (Transfer.INSTANCE.scoreTimes >= 3) return;

                // only look at the SAME command you scheduled
                if ( !shotYet ||  (currentShot != null && currentShot.isDone()) ) {

                    shotYet = true;
                    currentShot = Transfer.INSTANCE.kickBall();
                    currentShot.invoke(); // schedule next shot ONCE
                }
            }

            @Override
            public boolean isDone() {
                return Transfer.INSTANCE.scoreTimes >= 3;
            }
        };
    }
    public Command Test(){
            return new Command() {
                    Command inrCmd;
                    @Override
                    public void start () {
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
