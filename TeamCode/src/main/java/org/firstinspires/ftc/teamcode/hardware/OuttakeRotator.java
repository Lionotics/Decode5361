package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class OuttakeRotator extends Subsystem {

    private Servo hood;

    public  static double rotatorDefaultPosition = 0.05;

    public  static  double rotatorSecondPosition = 0.1;

    public static final double rotatorStep = 0.01;

    private  OuttakeRotator() {}

    public static OuttakeRotator INSTANCE = new OuttakeRotator();

    public void initialize() {
        hood = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Hood");

        hood.setPosition(rotatorDefaultPosition);
    }

    public InstantCommand rotateUp() {
        return new InstantCommand(()-> {
            hood.setPosition( hood.getPosition() + rotatorStep );
        });
    }

    public InstantCommand rotateDown() {
        return new InstantCommand(()-> {
            hood.setPosition( hood.getPosition() - rotatorStep );
        });
    }

    public InstantCommand setHoodToDefaultPosition() {
        return new InstantCommand(()-> {
            hood.setPosition( rotatorDefaultPosition );
        });
    }

    public InstantCommand setHoodToSecondPosition() {
        return new InstantCommand(()-> {
            hood.setPosition( rotatorSecondPosition );
        });
    }

    public InstantCommand setHoodPosition(double hoodPosition) {
        return new InstantCommand(()-> {
            hood.setPosition( hoodPosition );
        });
    }



    public  double getHoodPosition() {
        return hood.getPosition();
    }

}
