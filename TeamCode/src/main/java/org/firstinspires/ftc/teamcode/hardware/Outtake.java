package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToVelocity;

@Config
public class Outtake extends Subsystem {

    public  static double motorPower = 0.5;

    public  static  double kP = 0.01;
    public  static  double kI = 0.00;
    public  static  double kD = 0.00;

    public  static double motorVelocity = 60;

    private boolean motorRunning = false;


    private final PIDFController outtakeVelocityController = new PIDFController(
            kP,  // kP
            kI,   // kI
            kD    // kD
    );


    private  Outtake() {}

    public static Outtake INSTANCE = new Outtake();

    private MotorEx motorOuttakeRight;
    private MotorEx motorOuttakeLeft;

    private  MotorGroup flywheelGroup;




    public void initialize() {
        motorOuttakeRight = new MotorEx("flyWheelRight");
        motorOuttakeLeft = new MotorEx("flyWheelLeft");

        motorOuttakeLeft.reverse();
        motorOuttakeRight.reverse();

         flywheelGroup = new MotorGroup(motorOuttakeLeft, motorOuttakeRight);
    }

    public InstantCommand setPowerToMotorS(double i) {
        return new InstantCommand(()-> {
            //motorOuttakeRight.setPower(i*motorPower);
            flywheelGroup.setPower(i*motorPower);


        });
    }

    public Command handleMotor() {
        double targetTemp  = motorVelocity; // ignore direction
        if (!motorRunning) {
            motorRunning = true;

            return new RunToVelocity(
                    flywheelGroup,
                    targetTemp,
                    outtakeVelocityController,
                    this
            );
        } else{

            motorRunning = false;
            return  stopMotor();
        }

    }

    public Command stopMotor() {
        return new InstantCommand(() -> {
            flywheelGroup.setPower(0);
        });
    }

    public  double getMotorCurrentLeftVelocity() {
        return motorOuttakeLeft.getVelocity();
    }

    public  double getMotorCurrentRightVelocity() {
        return motorOuttakeRight.getVelocity();
    }


}
