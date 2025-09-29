package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@Config
@TeleOp(name = "YeshivaLeague2025Teleop", group = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Command driverControlled;

    public Teleop() {
        super(DriveTrain.INSTANCE);
    }

    private GamepadEx gp1;

    @Override
    public void onStartButtonPressed() {
        gp1 = gamepadManager.getGamepad1();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    @Override
    public void onUpdate() {

    }
}
