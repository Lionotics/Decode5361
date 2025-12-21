package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

@Config
@TeleOp(name = "5361Teleop", group = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Command driverControlled;

    public Teleop() {
        super(DriveTrain.INSTANCE,Intake.INSTANCE, Transfer.INSTANCE,Outtake.INSTANCE);

    }


    @Override
    public void onStartButtonPressed() {
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

        gp1.getDpadUp().setHeldCommand( ()-> Outtake.INSTANCE.raiseMotorVelocity() );
        gp1.getDpadDown().setHeldCommand( ()-> Outtake.INSTANCE.lowerMotorVelocity() );
    }

    @Override
    public void onUpdate() {

        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());
        telemetry.addData("Motor Outtake Target Velocity: ",  Outtake.motorVelocityTarget);
        telemetry.addData("Motor Velocity Is Higher (true if Higher, false if Lower): ",  Outtake.motorIsOnHigher);

        telemetry.update();
    }
}
