package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@TeleOp(name = "5361RedTeleop", group = "Teleop")
public class RedTeleop extends  TeleopParent{
    public RedTeleop() {
        super();
    }

    @Override
    public void onStartButtonPressed() {
        super.onStartButtonPressed();
        DriveTrain.INSTANCE.setGoalID(RED_TAG_ID);
    }

    @Override
    public  void onUpdate() {
        super.onUpdate();
    }

    @Override
    public  void onStop() {
        super.onStop();
    }



    }
