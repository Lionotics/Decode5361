package org.firstinspires.ftc.teamcode.pedroPathing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Config // Panels
public class PedroAutonomous extends OpMode {

    private Telemetry panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public static double startAngle = 90;

    public static double endY = 50;


    @Override
    public void init() {
        panelsTelemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(startAngle)));

        paths = new Paths(follower); // Build paths

        follower.update();

        panelsTelemetry.addLine("Status Initialized");
        panelsTelemetry.addData("Path State", pathState);
        panelsTelemetry.addData("X", follower.getPose().getX());
        panelsTelemetry.addData("Y", follower.getPose().getY());
        panelsTelemetry.addData("Heading", follower.getPose().getHeading());

        panelsTelemetry.update();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.addData("Path State", pathState);
        panelsTelemetry.addData("X", follower.getPose().getX());
        panelsTelemetry.addData("Y", follower.getPose().getY());
        panelsTelemetry.addData("Heading", follower.getPose().getHeading());
        panelsTelemetry.update();
    }

    public static class Paths {

        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(56.000, endY))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(startAngle), Math.toRadians(startAngle))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following Path1 once at the beginning
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                // Wait until the follower is done with the path
                if (!follower.isBusy()) {
                    // Path finished – you could start another path here
                    pathState = 2;
                }
                break;

            case 2:
                // Done – robot will just hold position
                break;
        }

        return pathState;
    }

}
