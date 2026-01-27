package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;

@TeleOp(name = "5361BlueTeleop", group = "Teleop")
public class BlueTeleop extends  TeleopParent{
    public BlueTeleop() {
        super();
    }

    @Override
    public void onStartButtonPressed() {
        super.onStartButtonPressed();
        DriveTrain.INSTANCE.setGoalID(BLUE_TAG_ID);
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
