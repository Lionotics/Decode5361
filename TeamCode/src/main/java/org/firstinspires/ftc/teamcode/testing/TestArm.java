package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.hardware.Arm;

@Config
@TeleOp(name = "YeshivaLeague2026TestArm", group = "Teleop")
public class TestArm extends NextFTCOpMode {

    public Command driverControlled;

    public TestArm() {
        // super(DriveTrain.INSTANCE);
        super(Arm.INSTANCE);
    }

    private GamepadEx gp1;


    @Override
    public void onStartButtonPressed() {
        gp1 = gamepadManager.getGamepad1();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gp1.getDpadUp().setPressedCommand(() -> Arm.INSTANCE.setPowerToMotor(1));
        gp1.getDpadDown().setPressedCommand(() -> Arm.INSTANCE.setPowerToMotor(-1));
        gp1.getDpadUp().setReleasedCommand(()-> Arm.INSTANCE.setPowerToMotor(0));
        gp1.getDpadDown().setReleasedCommand(()-> Arm.INSTANCE.setPowerToMotor(0));

        gp1.getA().setPressedCommand( () -> Arm.INSTANCE.setServoPosition(1));
        gp1.getB().setPressedCommand( () -> Arm.INSTANCE.setServoPosition(0));

        gp1.getX().setPressedCommand( () -> Arm.INSTANCE.setServoPositionNotOverriding(1));
        gp1.getY().setPressedCommand( () -> Arm.INSTANCE.setServoPositionNotOverriding(0));



    }

    @Override
    public void onUpdate() {
        telemetry.addData("Servo Position ",  Arm.INSTANCE.getServoPosition());
        telemetry.addData("Servo Target Position ",  Arm.INSTANCE.motorPositionTarget);


        telemetry.update();
    }

}