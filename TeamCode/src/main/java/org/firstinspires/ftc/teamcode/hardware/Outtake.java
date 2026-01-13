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

    public  static double motorVelocityStep = 1;

    public  static double motorPower = 0.5;

    public  static  double kP = 0.01;
    public  static  double kI = 0.00;
    public  static  double kD = 0.000;

    public  static double motorVelocityTargetLower = 1240;
    public  static double motorVelocityTargetHigher = 1600;

    public  static double motorVelocityTarget = motorVelocityTargetHigher;

    public  static  double motorVelocityThreashhold = 30;

    private boolean motorRunning = false;

    public  static  boolean motorIsOnHigher = true;


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
        return  -targetVelocity-20;
        //return  0.0418 * targetVelocity + 5;
    }


    public void initialize() {
        motorOuttakeRight = new MotorEx("flyWheelRight");
        motorOuttakeLeft = new MotorEx("flyWheelLeft");

        //  motorOuttakeLeft.reverse();
        //  motorOuttakeRight.reverse();

        flywheelGroup = new MotorGroup(motorOuttakeLeft, motorOuttakeRight);
        motorRunning = false;
        motorVelocityTarget = motorVelocityTargetHigher;
        motorIsOnHigher = true;
    }


    public double distanceToVelocity(double distance) {
        return  4.8 * distance + /*974*/ 1000;
    }

    public double distanceToHoodPosition(double distance) {
        //return  -0.0000089*distance*distance + 0.0023 * distance - 0.06;
                 return distance*distance*distance * 0.00000028  -0.0000776*distance*distance + 0.0073 * distance - 0.15;
    }




    public Command handleMotor(double WebCamDistance) {
        if (!motorRunning) {
            motorRunning = true;
            double targetTempRaw = distanceToVelocity(WebCamDistance);
            OuttakeRotator.INSTANCE.setHoodPosition( distanceToHoodPosition(WebCamDistance) );

            return  holdVelocity(targetTempRaw);
        } else{
            motorRunning = false;
            return  stopMotor();
        }

    }



    public Command holdVelocity(double targetTempRaw) {
        motorRunning = true;
        double targetTemp = targetVelocityToActualVelocity(targetTempRaw);
        motorVelocityTarget  = targetTempRaw;
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
        return new RunToVelocity(
                flywheelGroup,
                0,
                outtakeVelocityController,
                this
        );

        // return new InstantCommand(() -> {
        //   flywheelGroup.setPower(0);
        // });
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