package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class OuttakeRotator extends Subsystem {

    public Servo rotatorMotor;

    public  static double rotatorDefaultPosition = 0.2;
    public static final double rotatorStep = 0.01;

    private  OuttakeRotator() {}

    public static OuttakeRotator INSTANCE = new OuttakeRotator();

    public void initialize() {
        rotatorMotor = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Hood");

        rotatorMotor.setPosition(rotatorDefaultPosition);
    }

    public InstantCommand rotateUp() {
        return new InstantCommand(()-> {
            rotatorMotor.setPosition( rotatorMotor.getPosition() + rotatorStep );
        });
    }

    public InstantCommand rotateDown() {
        return new InstantCommand(()-> {
            rotatorMotor.setPosition( rotatorMotor.getPosition() - rotatorStep );
        });
    }
}
