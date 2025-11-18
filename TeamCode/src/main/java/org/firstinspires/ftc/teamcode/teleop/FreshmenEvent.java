package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.MotorForOpenHouse;

@Config
@TeleOp(name = "2025FreshmenEvent", group = "Teleop")
public class FreshmenEvent extends NextFTCOpMode {
    public Command driverControlled;

    public FreshmenEvent() {
        super(DriveTrain.INSTANCE);
    }

    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverControlled = DriveTrain.INSTANCE.Drive(gamepadManager.getGamepad1(), false);
        driverControlled.invoke();
        GamepadEx gp1 = gamepadManager.getGamepad1();
        GamepadEx gp2 = gamepadManager.getGamepad2();

        gp1.getDpadUp().setPressedCommand(() -> MotorForOpenHouse.INSTANCE.setPowerToMotorOpenhouse(1));
        gp1.getDpadDown().setPressedCommand(() -> MotorForOpenHouse.INSTANCE.setPowerToMotorOpenhouse(-1));
        gp1.getDpadUp().setReleasedCommand(()-> MotorForOpenHouse.INSTANCE.setPowerToMotorOpenhouse(0));
        gp1.getDpadDown().setReleasedCommand(()-> MotorForOpenHouse.INSTANCE.setPowerToMotorOpenhouse(0));

    }



}
