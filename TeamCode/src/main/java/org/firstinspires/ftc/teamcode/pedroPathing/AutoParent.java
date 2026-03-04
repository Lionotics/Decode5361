package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Outtake;
import org.firstinspires.ftc.teamcode.hardware.OuttakeRotator;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.hardware.Webcam;

import com.rowanmcalpin.nextftc.core.command.Command;

/**
 * AutoParent:
 * - owns the common NextFTC/Pedro lifecycle (onInit/onUpdate)
 * - initializes follower + telemetry + drivetrain brake
 * - provides hooks for child autos to define paths + state machine logic
 *
 * Child autos should override:
 *   - buildPaths()
 *   - autonomousPathUpdate()
 *   - (optional) getStartPose()
 *   - (optional) addSubclassTelemetry()
 */
@Config
public abstract class AutoParent extends PedroOpMode {

    public final int BLUE_TAG_ID = 20;
    public final int RED_TAG_ID = 24;

    // Panels telemetry
    protected Telemetry panelsTelemetry;

    // Pedro follower + state machine state
    protected Follower follower;
    protected int pathState = 0;

    // Common “turn then do something” helpers (children may use or ignore)
    public static boolean rebuildPaths = false;

    protected long turnStartMs = 0;
    protected double target = 0;

    protected  double turnMillisecondsWait = 2000;


    // Common command slot if your autos use it
    protected Command scoreCmd = null;

    protected  boolean webCamAtStartUpdatingEveryFrame = false;

    public AutoParent() {
        super(Intake.INSTANCE, Outtake.INSTANCE, Transfer.INSTANCE, Webcam.INSTANCE, OuttakeRotator.INSTANCE);
    }

    /** Override if your auto starts somewhere else. */
    protected Pose getStartPose() {
        return new Pose(0, 0, Math.toRadians(90));
    }

    /** Child defines its PathChains here (using the already-created follower). */
    protected abstract void buildPaths();

    /** Child’s autonomous FSM update; should read/write pathState and return it. */
    protected abstract int autonomousPathUpdate();

    /** Optional: child can add extra telemetry lines/data. */
    protected void addSubclassTelemetry() {}

    @Override
    public void onInit() {

        if (Webcam.ftcDashBoardTurnedOn) {
            FtcDashboard.getInstance().startCameraStream(Webcam.INSTANCE.getVisionPortal(), 30);
        }

        // Reset per-run state
        pathState = 0;
        scoreCmd = null;
        turnStartMs = 0;
        target = 0;

        super.onInit();

        panelsTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Create follower
        follower = Constants.createFollower(hardwareMap);

        // Force drivetrain motors to BRAKE at zero power (matches your current setup)
        DcMotorEx lf = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx lr = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx rf = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx rr = hardwareMap.get(DcMotorEx.class, "backRight");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        webCamAtStartUpdatingEveryFrame = Webcam.updateCameraEveryFrame;
        Webcam.updateCameraEveryFrame = true;


        // Start pose + paths
        follower.setStartingPose(getStartPose());
        buildPaths();

        follower.update();



        panelsTelemetry.addLine("Status: Initialized");
        panelsTelemetry.addData("Path State", pathState);
        panelsTelemetry.addData("X", follower.getPose().getX());
        panelsTelemetry.addData("Y", follower.getPose().getY());
        panelsTelemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update();




    }

    @Override
    public void onUpdate() {
        super.onUpdate();

        // If you use “rebuildPaths” from dashboard while init/running, this is the safest pattern:
        // rebuilds once and clears the flag.
        if (rebuildPaths) {
            buildPaths();
            rebuildPaths = false;
        }

        follower.update();
        pathState = autonomousPathUpdate();

        // Base telemetry
        panelsTelemetry.addData("Path State", pathState);
        panelsTelemetry.addData("X", follower.getPose().getX());
        panelsTelemetry.addData("Y", follower.getPose().getY());
        panelsTelemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.addData("Outtake Target Velocity", Outtake.motorVelocityTarget);
        panelsTelemetry.addData("Outtake Current Velocity", Outtake.INSTANCE.getMotorCurrentLeftVelocity());

        panelsTelemetry.addData("Is Intake Full", Intake.INSTANCE.isFull());

        if (Webcam.INSTANCE.seesTag()) {
            panelsTelemetry.addData("Goal Bearing: ", Webcam.INSTANCE.getBearing());

        } else {
                panelsTelemetry.addData("Goal Bearing: ", "Can't see goal." );

        }



        // Child telemetry
        addSubclassTelemetry();

        panelsTelemetry.update();

    }

    @Override
    public void onStop() {
        if (Webcam.ftcDashBoardTurnedOn) {
            FtcDashboard.getInstance().stopCameraStream();
        }
        Webcam.updateCameraEveryFrame = webCamAtStartUpdatingEveryFrame;

        super.onStop();
    }
}
