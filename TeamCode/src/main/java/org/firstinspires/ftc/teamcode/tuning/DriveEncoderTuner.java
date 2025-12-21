package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.gamepad.GamepadEx;

import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;

import org.firstinspires.ftc.teamcode.hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive encoder localization tuner using NextFTC + your DriveTrain.
 *
 * Buttons:
 *  - A: start/reset current test (forward/strafe/turn) or finish at the end.
 *  - B: lock in measurement for the current test.
 *  - X: move from FORWARD to STRAFE test.
 *  - Y: move from STRAFE to TURN test.
 */
@TeleOp(name = "Drive Encoder Tuner (NextFTC)", group = "Tuning")
public class DriveEncoderTuner extends NextFTCOpMode {

    // Distances/angles for the tests
    private static final double FORWARD_TEST_DISTANCE_IN = 60.0;
    private static final double STRAFE_TEST_DISTANCE_IN  = 60.0;
    private static final double FULL_TURN_RADIANS        = 2.0 * Math.PI;

    private enum Stage {
        FORWARD_PREP,
        FORWARD_DRIVE,
        FORWARD_DONE,
        STRAFE_PREP,
        STRAFE_DRIVE,
        STRAFE_DONE,
        TURN_PREP,
        TURN_DRIVE,
        TURN_DONE,
        COMPLETE
    }

    // NextFTC / driving
    private Command driverControlled;
    public GamepadEx gp1;

    // Encoders (raw motors for tick counts)
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private DriveEncoderConstants encConstants;

    // State for tuning
    private Stage stage = Stage.FORWARD_PREP;
    private Double tunedForward = null;
    private Double tunedStrafe  = null;
    private Double tunedTurn    = null;

    public DriveEncoderTuner() {
        // Use your DriveTrain subsystem with NextFTC
        super(DriveTrain.INSTANCE);
    }

    @Override
    public void onStartButtonPressed() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Use the constants from Constants.java
        encConstants = Constants.localizerConstants;

        // Grab the drive motors using the same names the localizer uses
        leftFront  = hardwareMap.get(DcMotorEx.class, encConstants.leftFrontMotorName);
        leftRear   = hardwareMap.get(DcMotorEx.class, encConstants.leftRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, encConstants.rightFrontMotorName);
        rightRear  = hardwareMap.get(DcMotorEx.class, encConstants.rightRearMotorName);

        // Brake when stopped so position is stable
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetAllEncoders();

        // Set up normal driving using your DriveTrain + NextFTC
        gp1 = gamepadManager.getGamepad1();
        driverControlled = DriveTrain.INSTANCE.Drive(gp1, false);
        driverControlled.invoke();

        // --------- BUTTON BINDINGS (no prevA/prevB needed) ---------

        // A: start/reset the current test or finish at the end
        gp1.getA().setPressedCommand( () -> {
            return  new InstantCommand( ()-> {

            switch (stage) {
                case FORWARD_PREP:
                    resetAllEncoders();
                    stage = Stage.FORWARD_DRIVE;
                    break;
                case STRAFE_PREP:
                    resetAllEncoders();
                    stage = Stage.STRAFE_DRIVE;
                    break;
                case TURN_PREP:
                    resetAllEncoders();
                    stage = Stage.TURN_DRIVE;
                    break;
                case TURN_DONE:
                    stage = Stage.COMPLETE;
                    break;
                default:
                    // no-op in other stages
                    break;
            }

            } );
        });

        // B: lock in measurement for the current test
        gp1.getB().setPressedCommand(() -> {


            return  new InstantCommand( ()-> {


            int lfTicks = leftFront.getCurrentPosition();
            int rfTicks = rightFront.getCurrentPosition();
            int lrTicks = leftRear.getCurrentPosition();
            int rrTicks = rightRear.getCurrentPosition();

            switch (stage) {
                case FORWARD_DRIVE: {
                    int sumTicks = lfTicks + rfTicks + lrTicks + rrTicks;
                    if (sumTicks != 0) {
                        tunedForward = FORWARD_TEST_DISTANCE_IN / (double) sumTicks;
                        stage = Stage.FORWARD_DONE;
                    }
                    break;
                }

                case STRAFE_DRIVE: {
                    int lateralTicks = -lfTicks + rfTicks + lrTicks - rrTicks;
                    if (lateralTicks != 0) {
                        tunedStrafe = STRAFE_TEST_DISTANCE_IN / (double) Math.abs(lateralTicks);
                        stage = Stage.STRAFE_DONE;
                    }
                    break;
                }

                case TURN_DRIVE: {
                    int turnTicksCombo = -lfTicks + rfTicks - lrTicks + rrTicks;
                    double widthPlusLength = encConstants.robot_Width + encConstants.robot_Length;
                    if (turnTicksCombo != 0 && widthPlusLength != 0) {
                        tunedTurn = (FULL_TURN_RADIANS * widthPlusLength) /
                                (double) Math.abs(turnTicksCombo);
                        stage = Stage.TURN_DONE;
                    }
                    break;
                }

                default:
                    break;
            }
            } );
        });

        // X: move from FORWARD_DONE → STRAFE_PREP
        gp1.getX().setPressedCommand(() -> {
                return  new InstantCommand(  ()->{
                if (stage == Stage.FORWARD_DONE) {
                    resetAllEncoders();
                    stage = Stage.STRAFE_PREP;
                }
            });
        });


        // Y: move from STRAFE_DONE → TURN_PREP
        gp1.getY().setPressedCommand( () ->
        {
            return new InstantCommand(  ()-> {
                if (stage == Stage.STRAFE_DONE) {
                    resetAllEncoders();
                    stage = Stage.TURN_PREP;
                }

            }
            );
        });

        telemetry.addLine("Drive Encoder Tuner (NextFTC + DriveTrain)");
        telemetry.addLine("Use sticks to drive as normal.");
        telemetry.addLine("Buttons: A/B/X/Y step through tuning stages.");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        // Just read ticks and show info; stage changes are handled by button commands
        int lfTicks = leftFront.getCurrentPosition();
        int rfTicks = rightFront.getCurrentPosition();
        int lrTicks = leftRear.getCurrentPosition();
        int rrTicks = rightRear.getCurrentPosition();

        telemetry.addData("Stage", stage);
        telemetry.addData("lfTicks", lfTicks);
        telemetry.addData("rfTicks", rfTicks);
        telemetry.addData("lrTicks", lrTicks);
        telemetry.addData("rrTicks", rrTicks);

        switch (stage) {
            // ---------- FORWARD ----------
            case FORWARD_PREP:
                telemetry.addLine("FORWARD tuning:");
                telemetry.addLine("- Put robot on a taped START line, facing forward.");
                telemetry.addLine("- You will drive exactly " + FORWARD_TEST_DISTANCE_IN + " inches forward.");
                telemetry.addLine("Press A to reset encoders and begin FORWARD test.");
                break;

            case FORWARD_DRIVE: {
                telemetry.addLine("FORWARD tuning:");
                telemetry.addLine("- Drive forward until the FRONT of the robot reaches");
                telemetry.addLine("  the second line (" + FORWARD_TEST_DISTANCE_IN + " in from start).");
                telemetry.addLine("Press B once it's exactly on the line to lock in value.");

                int sumTicks = lfTicks + rfTicks + lrTicks + rrTicks;
                telemetry.addData("sumTicks (forward)", sumTicks);
                if (sumTicks != 0) {
                    double forwardMultiplier =
                            FORWARD_TEST_DISTANCE_IN / (double) sumTicks;
                    telemetry.addData("Live forwardTicksToInches (preview)",
                            "%.8f", forwardMultiplier);
                } else {
                    telemetry.addLine("Move the robot forward so encoder counts change...");
                }
                break;
            }

            case FORWARD_DONE:
                telemetry.addLine("FORWARD tuning DONE.");
                telemetry.addData("forwardTicksToInches",
                        tunedForward != null ? String.format("%.8f", tunedForward) : "N/A");
                telemetry.addLine("Paste into Constants.localizerConstants:");
                if (tunedForward != null) {
                    telemetry.addLine(".forwardTicksToInches(" +
                            String.format("%.8f", tunedForward) + ")");
                }
                telemetry.addLine("Press X to start STRAFE tuning.");
                break;

            // ---------- STRAFE ----------
            case STRAFE_PREP:
                telemetry.addLine("STRAFE tuning:");
                telemetry.addLine("- Put robot back on the START line, centered.");
                telemetry.addLine("- You will STRAFE exactly " + STRAFE_TEST_DISTANCE_IN + " inches.");
                telemetry.addLine("Press A to reset encoders and begin STRAFE test.");
                break;

            case STRAFE_DRIVE: {
                telemetry.addLine("STRAFE tuning:");
                telemetry.addLine("- Strafe sideways until the robot travels exactly");
                telemetry.addLine("  " + STRAFE_TEST_DISTANCE_IN + " inches.");
                telemetry.addLine("Press B once it's exactly on the second line.");

                int lateralTicks = -lfTicks + rfTicks + lrTicks - rrTicks;
                telemetry.addData("lateralTicks (raw)", lateralTicks);

                if (lateralTicks != 0) {
                    double strafeMultiplier =
                            STRAFE_TEST_DISTANCE_IN / (double) Math.abs(lateralTicks);
                    telemetry.addData("Live strafeTicksToInches (preview)",
                            "%.8f", strafeMultiplier);
                } else {
                    telemetry.addLine("Strafe until encoder counts change...");
                }
                break;
            }

            case STRAFE_DONE:
                telemetry.addLine("STRAFE tuning DONE.");
                telemetry.addData("strafeTicksToInches",
                        tunedStrafe != null ? String.format("%.8f", tunedStrafe) : "N/A");
                telemetry.addLine("Paste into Constants.localizerConstants:");
                if (tunedStrafe != null) {
                    telemetry.addLine(".strafeTicksToInches(" +
                            String.format("%.8f", tunedStrafe) + ")");
                }
                telemetry.addLine("Press Y to start TURN tuning.");
                break;

            // ---------- TURN ----------
            case TURN_PREP:
                telemetry.addLine("TURN tuning:");
                telemetry.addLine("- Put robot on a heading reference line.");
                telemetry.addLine("- You will rotate it EXACTLY 360° back to this heading.");
                telemetry.addLine("Press A to reset encoders and begin TURN test.");
                break;

            case TURN_DRIVE: {
                telemetry.addLine("TURN tuning:");
                telemetry.addLine("- Rotate the robot exactly 360° (back to the line).");
                telemetry.addLine("Press B when you are back on the line.");

                int turnTicksCombo = -lfTicks + rfTicks - lrTicks + rrTicks;
                double widthPlusLength =
                        encConstants.robot_Width + encConstants.robot_Length;

                telemetry.addData("turnTicksCombo (raw)", turnTicksCombo);
                telemetry.addData("robot_Width + robot_Length", widthPlusLength);

                if (turnTicksCombo != 0 && widthPlusLength != 0) {
                    double turnMultiplier =
                            (FULL_TURN_RADIANS * widthPlusLength) /
                                    (double) Math.abs(turnTicksCombo);
                    telemetry.addData("Live turnTicksToInches (preview)",
                            "%.8f", turnMultiplier);
                } else {
                    telemetry.addLine("Rotate until counts change and width/length are set.");
                }
                break;
            }

            case TURN_DONE:
                telemetry.addLine("TURN tuning DONE.");
                telemetry.addData("turnTicksToInches",
                        tunedTurn != null ? String.format("%.8f", tunedTurn) : "N/A");
                telemetry.addLine("Paste into Constants.localizerConstants:");
                if (tunedTurn != null) {
                    telemetry.addLine(".turnTicksToInches(" +
                            String.format("%.8f", tunedTurn) + ")");
                }
                telemetry.addLine("Press A to view full summary.");
                break;

            case COMPLETE:
                telemetry.addLine("ALL TUNING COMPLETE.");
                telemetry.addLine("Put these into Constants.localizerConstants:");
                if (tunedForward != null) {
                    telemetry.addLine(".forwardTicksToInches(" +
                            String.format("%.8f", tunedForward) + ");");
                }
                if (tunedStrafe != null) {
                    telemetry.addLine(".strafeTicksToInches(" +
                            String.format("%.8f", tunedStrafe) + ");");
                }
                if (tunedTurn != null) {
                    telemetry.addLine(".turnTicksToInches(" +
                            String.format("%.8f", tunedTurn) + ");");
                }
                telemetry.addLine("You can still drive around with your normal controls.");
                break;
        }

        telemetry.update();
    }

    private void resetAllEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
