package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.Outtake;






@Config
@TeleOp(name = "2025OuttakeTesting", group = "Teleop")
public class TestOuttake extends NextFTCOpMode {

    public TestOuttake() {
        super(Outtake.INSTANCE);
    }

    @Override
    public void onStartButtonPressed() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        GamepadEx gp1 = gamepadManager.getGamepad1();
        GamepadEx gp2 = gamepadManager.getGamepad2();



        gp1.getB().setPressedCommand(() -> Outtake.INSTANCE.handleMotor());
        gp1.getA().setPressedCommand(() -> Outtake.INSTANCE.stopMotor());


    }

    @Override
    public void onUpdate() {
        telemetry.addData("Motor Outtake Current Velocity: ",  Outtake.INSTANCE.getMotorCurrentLeftVelocity());
        telemetry.update();
    }



}
