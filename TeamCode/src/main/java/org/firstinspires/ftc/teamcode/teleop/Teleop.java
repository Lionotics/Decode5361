package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Feet;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;

@Config
@TeleOp(name = "5361Teleop", group = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Command driverControlled;

    public Teleop() {
        super(DriveTrain.INSTANCE,Intake.INSTANCE,Feet.INSTANCE,Outtake.INSTANCE);

    }


    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverControlled = DriveTrain.INSTANCE.Drive(gamepadManager.getGamepad1(), false);
        driverControlled.invoke();
        GamepadEx gp1 = gamepadManager.getGamepad1();
        GamepadEx gp2 = gamepadManager.getGamepad2();

        gp1.getRightBumper().setPressedCommand(() -> Intake.INSTANCE.eat());
        gp1.getLeftBumper().setPressedCommand(() -> Intake.INSTANCE.spit());

        gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.handleMotor());
        //gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.setPowerToMotorS(1));
        //gp1.getB().setPressedCommand(() -> Outtake.INSTANCE.stopMotor());

        gp1.getX().setPressedCommand(() -> Feet.INSTANCE.kickBall());
        gp1.getY().setPressedCommand(() -> Intake.INSTANCE.loadBall(0.5));


    }

    @Override
    public void onUpdate() {

        telemetry.addData("Motor Outtake Left Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.addData("Motor Outtake Right Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentRightVelocity());

        telemetry.update();
    }
}
