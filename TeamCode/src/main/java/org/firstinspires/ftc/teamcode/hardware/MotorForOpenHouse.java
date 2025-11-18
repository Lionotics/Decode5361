package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

public class MotorForOpenHouse {

    public MotorEx motor;

    private MotorForOpenHouse() {

    }

    public void initialize() {
        motor = new MotorEx("Motor");
    }

    public static MotorForOpenHouse INSTANCE = new MotorForOpenHouse();

    public InstantCommand setPowerToMotor(double i) {
        return new InstantCommand(()-> {
            motor.setPower(i);
        });
    }

}
