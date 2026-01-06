package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.ForcedParallelCommand;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.OuttakeRotator;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.Webcam;

@Config
@TeleOp(name = "5361Teleop", group = "Teleop")
public class Teleop extends NextFTCOpMode {

    public  static double delayInitial = 0;

    public Command driverControlled;

    public Teleop() {
        super(DriveTrain.INSTANCE,Intake.INSTANCE, Transfer.INSTANCE,Outtake.INSTANCE, OuttakeRotator.INSTANCE, Webcam.INSTANCE);

    }


    @Override
    public void onStartButtonPressed() {
        FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverControlled = DriveTrain.INSTANCE.Drive(gamepadManager.getGamepad1(), false);
        driverControlled.invoke();
        GamepadEx gp1 = gamepadManager.getGamepad1();
       // GamepadEx gp2 = gamepadManager.getGamepad2();




        /*gp1.getRightTrigger().setPressedCommand(new Function1<Float, Command>() {
            @Override
            public Command invoke(Float value) {
                // value is the analog trigger value (0.0–1.0); you can ignore it
                Intake.INSTANCE.eat();   // side effect
                return new NullCommand();        // schedules a do-nothing command
            }
        });

        gp1.getLeftTrigger().setPressedCommand(new Function1<Float, Command>() {
            @Override
            public Command invoke(Float value) {
                // value is the analog trigger value (0.0–1.0); you can ignore it
                Intake.INSTANCE.spit();   // side effect
                return new NullCommand();        // schedules a do-nothing command
            }
        }); */

        gp1.getA().setPressedCommand(() -> Transfer.INSTANCE.kickBall());

        gp1.getB().setPressedCommand(() -> Intake.INSTANCE.eat());


        gp1.getRightBumper().setPressedCommand(() -> Outtake.INSTANCE.handleMotor( Outtake.motorVelocityTarget ));

        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.spit());



        gp1.getDpadLeft().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToLower() );
        gp1.getDpadRight().setPressedCommand( ()->Outtake.INSTANCE.MotorVelocityToHigher() );

       // gp1.getDpadUp().setHeldCommand( ()-> Outtake.INSTANCE.raiseMotorVelocity() );
       // gp1.getDpadDown().setHeldCommand( ()-> Outtake.INSTANCE.lowerMotorVelocity() );

         gp1.getDpadUp().setHeldCommand( ()-> OuttakeRotator.INSTANCE.setHoodToDefaultPosition() );
         gp1.getDpadDown().setHeldCommand( ()-> OuttakeRotator.INSTANCE.setHoodToSecondPosition() );


        gp1.getLeftBumper().setPressedCommand( ()-> autoScore() );
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

        // Update intake fullness detection once per loop
        Intake.INSTANCE.updateFullDetection();

        telemetry.addData("Intaking Boolean", Intake.INSTANCE.intaking);
        telemetry.addData("Intake Vel Raw (tps)", Intake.INSTANCE.getRawVelocityTps());
        telemetry.addData("Intake Vel Filtered (tps)", Intake.INSTANCE.getFilteredVelocityTps());
        telemetry.addData("INTAKE FULL (3 balls)", Intake.INSTANCE.isFull() ? "YES" : "NO");




        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);
        telemetry.addData("Motor Velocity Is Higher (true if Higher, false if Lower): ",  Outtake.motorIsOnHigher);


        telemetry.addData("Score Times", Transfer.INSTANCE.scoreTimes);
        telemetry.addData("Temp", temp);


        Webcam.INSTANCE.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public  void onStop() {
        Webcam.INSTANCE.close();
        FtcDashboard.getInstance().stopCameraStream();
    }



    public Command autoScore() {
        Transfer.INSTANCE.scoreTimes = 0;
        return  new SequentialGroup(
                DriveTrain.INSTANCE.faceBlueGoal,
                new ForcedParallelCommand(Outtake.INSTANCE.holdVelocity(Outtake.motorVelocityTarget)  ),
                new Delay(delayInitial),
                score3Times(),
                Outtake.INSTANCE.stopMotor()
        );
    }



    public Command score3Times(){
        return new Command() {
            private Command currentShot;

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
                if ( Transfer.INSTANCE.scoreTimes == 0 ||  (currentShot != null && currentShot.isDone()) ) {
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



}
