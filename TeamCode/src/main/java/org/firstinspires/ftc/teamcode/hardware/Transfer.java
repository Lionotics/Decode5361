package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

@Config
public class Transfer extends Subsystem {
    public Servo kicker;
    public Servo protector;

    public  static double kickerPosition1 = 0.08;
    public  static double protectorPosition1 = 0.7;
    public  static double kickerPosition2 = 0.25;
    public  static double protectorPosition2 = 0.4;

    public static double kickDelaySeconds = 0.13;
    public static double preReturnDelaySeconds = 0.1;
    public static double postReturnDelaySeconds = 0.0;

    public static  double loadDelaySecond = 0.4;

    public static double shootDelaySeconds = 0.4;

    public int scoreTimes = 0;



    private Transfer() {}

    public static Transfer INSTANCE = new Transfer();




    public void initialize() {
        kicker = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Kicker");
        protector = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, "Protector");

        kicker.setPosition(kickerPosition1);
        protector.setPosition(protectorPosition1);

        scoreTimes = 0;
    }

    public InstantCommand keepBall() {
        return new InstantCommand(()-> {
            protector.setPosition(protectorPosition1);
            kicker.setPosition(kickerPosition1);
        });
    }

    public Command kickBall() {
        if (Outtake.INSTANCE.getMotorCurrentLeftVelocity() != 0 || Outtake.INSTANCE.getMotorCurrentRightVelocity() != 0) {
            return new SequentialGroup(
                    new WaitUntil(() -> Outtake.INSTANCE.flywheelReady(Outtake.motorVelocityTarget)),
                    new Delay(shootDelaySeconds),
                    new InstantCommand(() -> protector.setPosition(protectorPosition2)),
                    new Delay(kickDelaySeconds),
                    new InstantCommand(() -> kicker.setPosition(kickerPosition2)),
                    new Delay(preReturnDelaySeconds),
                    keepBall(),
                    new Delay(postReturnDelaySeconds),
                    Intake.INSTANCE.loadBall(loadDelaySecond),
                    new InstantCommand(() -> scoreTimes += 1)
            );
        } else {
            return  new NullCommand();
        }

    }




}
