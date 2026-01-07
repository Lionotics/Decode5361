package org.firstinspires.ftc.teamcode.hardware;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import androidx.annotation.NonNull;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;


@Config
public class DriveTrain extends Subsystem {
    public static final DriveTrain INSTANCE = new DriveTrain();
    private DriveTrain() {
    }
    private MotorEx frontLeft, frontRight, backLeft, backRight;

    public GoBildaPinpointDriver odometry;



    private MotorEx[] motors;
    private IMU imu;

    public void initialize() {
      //  frontLeft = new MotorEx("frontLeft");
       // frontRight = new MotorEx("frontRight");
       // backLeft = new MotorEx("backLeft");
       // backRight = new MotorEx("backRight");

        frontLeft = new MotorEx("backLeft");
        frontRight = new MotorEx("backRight");
        backLeft = new MotorEx("frontLeft");
        backRight = new MotorEx("frontRight");


        odometry = OpModeData.INSTANCE.getHardwareMap().get(GoBildaPinpointDriver.class, "Odometry");



        frontLeft.reverse();
        backLeft.reverse();

        motors = new MotorEx[]{frontLeft, frontRight, backLeft, backRight};

        initIMU(OpModeData.INSTANCE.getHardwareMap());
    }

    public void initIMU(HardwareMap hwMap) {
        // Retrieve the IMU from the hardware map
        imu = hwMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        odometry.resetPosAndIMU();
    }

    public Command Drive(GamepadEx gamepad, boolean robotOreinted) {
        return new MecanumDriverControlled(motors, gamepad, robotOreinted, imu);
    }

    public void setTurnPower(double turn) {
        turn = Range.clip(turn, -1.0, 1.0);

        // Turn in place: left side forward, right side backward (signs may be flipped if you prefer)
        frontLeft.setPower(turn);
        backLeft.setPower(turn);
        frontRight.setPower(-turn);
        backRight.setPower(-turn);
    }

    public void stopDrive() {
        setTurnPower(0.0);
    }





    final int BLUE_GOAL_TAG_ID = 20;

    // Tune these:

    public static double desiredTilt = 0;
    public static double turnSpeedWhenInRange = 0.23;       // turning proportional gain
    final double maxTurn = 0.45;   // cap turn power
    final double minTurn = 0.08;   // minimum to overcome friction
    public static double deadbandDeg = 2.0;
    public static long timeoutMs = 25000;

    public  static double turnPowerValue = 0.37;

    // tiny “mutable holders” for lambdas
    final long[] startTime = new long[1];
    final boolean[] sawTag = new boolean[1];
    final double[] lastErrorDeg = new double[1];

    public Command faceBlueGoal = new LambdaCommand()
            .setStart(() -> {
                startTime[0] = System.currentTimeMillis();
                sawTag[0] = false;
                lastErrorDeg[0] = 999;
            })
            .setUpdate(() -> {
                AprilTagDetection d = Webcam.INSTANCE.getDetectionById(BLUE_GOAL_TAG_ID);

                // If tag not visible, optionally slow-spin to “hunt” (still not moving closer)
                double error = 0;
                if (!(d == null || d.ftcPose == null)  ) {
                    sawTag[0] = true;
                    error = desiredTilt -d.ftcPose.bearing;   // degrees
                    lastErrorDeg[0] =   error;
                    if (Math.abs(lastErrorDeg[0]) < deadbandDeg) {
                        DriveTrain.INSTANCE.setTurnPower(0.0);
                        return;
                    }
                }


                if (  (d == null || d.ftcPose == null) ) {
                    DriveTrain.INSTANCE.setTurnPower(turnPowerValue);
                    return;
                }


                // IMPORTANT:
                // For “face the tag”, you almost always want BEARING (center the tag).
                // If you truly meant “match tag plane rotation”, use d.ftcPose.yaw instead.


                double turn = 0;
                if (error > 0) {
                    turn = turnSpeedWhenInRange;
                } else {
                    turn = -turnSpeedWhenInRange;
                }



                DriveTrain.INSTANCE.setTurnPower(turn);
            })
            .setIsDone(() -> {
                long elapsed = System.currentTimeMillis() - startTime[0];
                if (elapsed > timeoutMs) {
                      return true;
                }
                if (!sawTag[0]) {
                    return false;
                } // keep hunting until timeout
                return Math.abs( lastErrorDeg[0])  < deadbandDeg;
            })
            .setStop(interrupted -> {
                DriveTrain.INSTANCE.stopDrive();
                // resume normal teleop drive when done (or interrupted)
            });

}


