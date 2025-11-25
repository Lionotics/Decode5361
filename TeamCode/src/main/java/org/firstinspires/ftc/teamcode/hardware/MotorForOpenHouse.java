package org.firstinspires.ftc.teamcode.hardware;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

public class MotorForOpenHouse extends Subsystem {

    public MotorEx motor;

    private MotorForOpenHouse() {

    }

    public void initialize() {
        motor = new MotorEx("Motor");
    }

    public static MotorForOpenHouse INSTANCE = new MotorForOpenHouse();

    public InstantCommand setPowerToMotorOpenhouse(double i) {
        return new InstantCommand(()-> {
            motor.setPower(i);
        });
    }

}
