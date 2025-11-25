package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


@Config
public class Outtake extends Subsystem {

    public  static double motorPower = 0.5;
    public  static double motorVelocity = 600;


    private  Outtake() {}

    public static Outtake INSTANCE = new Outtake();

    public MotorEx motorOuttake;


    public void initialize() {
        motorOuttake = new MotorEx("Outtake");
    }

    public InstantCommand setPowerToMotorOuttake(double i) {
        return new InstantCommand(()-> {
            motorOuttake.setPower(i*motorPower);
        });
    }

    public InstantCommand setVelocityOfMotor(int direction) {
        return new InstantCommand(()-> {
            motorOuttake.setVelocity(motorVelocity*direction);
        });
    }
}
