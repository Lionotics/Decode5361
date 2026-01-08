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
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldVelocity;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;


import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToVelocity;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.jetbrains.annotations.NotNull;

@Config
public class Outtake extends Subsystem {

    public  static double motorVelocityStep = 1;

    public  static double motorPower = 0.5;

    public  static  double kP = 0.01;
    public  static  double kI = 0.005;
    public  static  double kD = 0.000;

    public  static double motorVelocityTargetLower = 1240;
    public  static double motorVelocityTargetHigher = 1600;

    public  static double motorVelocityTarget = motorVelocityTargetHigher;

    public  static  double motorVelocityThreashhold = 30;

    private boolean motorRunning = false;

    public  static  boolean motorIsOnHigher = true;

    public  static double velocityBooster = 0;




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

    public double targetVelocityToActualVelocity(double targetVelocity) {
        return  -targetVelocity-velocityBooster;
        //return  0.0418 * targetVelocity + 5;
    }


    public void initialize() {
        //motorOuttakeRight = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "flyWheelRight");
        //motorOuttakeLeft = OpModeData.INSTANCE.getHardwareMap().get(DcMotorEx.class, "flyWheelLeft");

      //  motorOuttakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       // motorOuttakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        motorOuttakeRight = new MotorEx("flyWheelRight");
        motorOuttakeLeft = new MotorEx("flyWheelLeft");




        flywheelGroup = new MotorGroup(motorOuttakeLeft, motorOuttakeRight);

        motorRunning = false;
        motorVelocityTarget = motorVelocityTargetHigher;
        motorIsOnHigher = true;
    }

    public InstantCommand setPowerToMotorS(double i) {
        return new InstantCommand(()-> {
            //motorOuttakeRight.setPower(i*motorPower);
            flywheelGroup.setPower(i*motorPower);
        });
    }

    public Command handleMotor(double targetTempRaw ) {
        double targetTemp = targetVelocityToActualVelocity(targetTempRaw);

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

    @NotNull
    @Override
    public Command getDefaultCommand() {
        // If we're "on", hold the last velocity; if we're "off", keep power at 0 (coast).
        if (motorRunning) {
            return new HoldVelocity(flywheelGroup, outtakeVelocityController, this);
        }
        return new SetPower(flywheelGroup, 0.0, this);
    }




    public Command goToVelocity(double targetTempRaw) {
        double targetTemp = targetVelocityToActualVelocity(targetTempRaw);
        motorRunning = true;
        // Runs the controller forever until interrupted by another Outtake command.
        return new RunToVelocity(
                        flywheelGroup,
                        targetTemp,
                        outtakeVelocityController,
                        this
                );
    }

    public   boolean flywheelReady(double targetRaw) {
        double avg = (getMotorCurrentLeftVelocity() + getMotorCurrentRightVelocity()) / 2.0;
        return Math.abs(targetRaw - avg) < motorVelocityThreashhold;
    }


    public Command stopMotor() {
        motorRunning = false;
        return new SetPower(flywheelGroup, 0.0, this);
    }


    public Command raiseMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityTarget += motorVelocityStep;
        });
    }


    public Command lowerMotorVelocity() {
        return new InstantCommand(() -> {
            motorVelocityTarget -= motorVelocityStep;
        });
    }


    public Command MotorVelocityToHigher() {
        return new InstantCommand(() -> {
            if (true || !motorIsOnHigher) {
                //  motorVelocityTargetLower = motorVelocityCurrent;
                motorVelocityTarget = motorVelocityTargetHigher;
                motorIsOnHigher = true;
            }
        });
    };

    public Command MotorVelocityToLower() {
        return new InstantCommand(() -> {
            if (true || motorIsOnHigher) {
                // motorVelocityTargetHigher = motorVelocityCurrent;
                motorVelocityTarget = motorVelocityTargetLower;
                motorIsOnHigher = false;
            }
        });
    };

    public  double getMotorCurrentLeftVelocity() {
        return -motorOuttakeLeft.getVelocity();
    }

    public  double getMotorCurrentRightVelocity() {
        return -motorOuttakeRight.getVelocity();
    }


}
