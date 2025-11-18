package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import java.util.Collections; // for Collections.singleton(...)

public class Arm extends Subsystem {
    public MotorEx motor;
    public Servo servo;

    private Arm() {

    }

    public boolean initRan  = false;

    public void initialize() {
        motor = new MotorEx("Motor");
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Servo");
        initRan = true;

    }

    public static Arm INSTANCE = new Arm();


    public  double motorPositionTarget = 0;

    public InstantCommand setPowerToMotor(double i) {
        return new InstantCommand(()-> {
            motor.setPower(i);
        });
    }

    public InstantCommand setServoPosition(double p) {
        return new InstantCommand(()-> {
            servo.setPosition(p);
        });
    }

    public Command setServoPositionNotOverriding(double p) {
        return  new InstantCommand( ()-> {
            while (servo.getPosition() != motorPositionTarget) {

            }
            servo.setPosition(p);
            motorPositionTarget = p;
        });


    }

    public  double getServoPosition() {
        return  servo.getPosition();
    }








}
