package org.firstinspires.ftc.teamcode.hardware;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

public class Intake extends Subsystem {


    public MotorEx motor;

    private Intake() {

    }

    public void initialize() {
        motor = new MotorEx("Intake");
    }

    public static Intake INSTANCE = new Intake();

    public InstantCommand setPowerToIntake(double i) {
        return new InstantCommand(()-> {
            motor.setPower(i);
        });
    }

    public InstantCommand eat() {
        return new InstantCommand(()-> {
            if (motor.getPower() != 0) {
                motor.setPower(0);
            } else {
                motor.setPower(-1);

            }
        });
    }

    public InstantCommand spit() {
        return new InstantCommand(()-> {
            if (motor.getPower() != 0) {
                motor.setPower(0);
            } else {
                motor.setPower(1);

            }
        });
    }

    public Command loadBall( double loadDelaySecond ) {
        return new SequentialGroup(
                new InstantCommand(() -> motor.setPower(-1) ),
                new Delay(loadDelaySecond),
                new InstantCommand(() ->  motor.setPower(0) )

        );

    }




}
