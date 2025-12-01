package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class Feet extends Subsystem {
    public Servo kicker;
    public Servo protector;

    public  static double kickerPosition1 = 0.1;
    public  static double protectorPosition1 = 0.7;
    public  static double kickerPosition2 = 0.4;
    public  static double protectorPosition2 = 0.4;

    public static double kickDelaySeconds = 0.13;
    public static double preReturnDelaySeconds = 0.4;
    public static double postReturnDelaySeconds = 0.4;

    public static  double loadDelaySecond = 0.5;


    private  Feet() {}

    public static Feet INSTANCE = new Feet();




    public void initialize() {
        kicker = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Kicker");
        protector = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Protector");

        kicker.setPosition(kickerPosition1);
        protector.setPosition(protectorPosition1);
    }

    public InstantCommand keepBall() {
        return new InstantCommand(()-> {
            protector.setPosition(protectorPosition1);
            kicker.setPosition(kickerPosition1);
        });
    }

    public Command kickBall() {
        // protector moves, wait, then kicker moves
        return new SequentialGroup(
                new InstantCommand(() -> protector.setPosition(protectorPosition2)),
                new Delay(kickDelaySeconds),
                new InstantCommand(() -> kicker.setPosition(kickerPosition2)),
                new Delay(preReturnDelaySeconds),
                keepBall(),
                new Delay(postReturnDelaySeconds),
                Intake.INSTANCE.loadBall(loadDelaySecond)
                );
    }




}
