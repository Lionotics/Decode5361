package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToVelocity;

@Config
public class Outtake extends Subsystem {

    public  static double motorPower = 0.5;

    public  static  double kP = 0.01;
    public  static  double kI = 0.00;
    public  static  double kD = 0.00;


    public  static double motorVelocity = 700;


    private final PIDFController outtakeVelocityController = new PIDFController(
            kP,  // kP
            kI,   // kI
            kD    // kD
    );


    private  Outtake() {}

    public static Outtake INSTANCE = new Outtake();

    private MotorEx motorOuttake;


    public void initialize() {
        motorOuttake = new MotorEx("Outtake");
    }

    public InstantCommand setPowerToMotorOuttake(double i) {
        return new InstantCommand(()-> {
            motorOuttake.setPower(i*motorPower);
        });
    }

    public Command setVelocityOfMotor() {
        double targetTemp  = -motorVelocity; // ignore direction

        return new RunToVelocity(
                motorOuttake,
                targetTemp,      // target velocity (ticks/sec)
                outtakeVelocityController,
                this                            // the Outtake subsystem
        );

    }

    public Command stopMotor() {

        return new RunToVelocity(
                motorOuttake,
                0,      // target velocity (ticks/sec)
                outtakeVelocityController,
                this                            // the Outtake subsystem
        );

    }

    public  double getMotorCurrentVelocity() {
        return  -1 * motorOuttake.getVelocity();
    }
}
