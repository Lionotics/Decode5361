package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import java.util.ArrayList;
import java.util.List;

public class Webcam extends Subsystem {

    public static final Webcam INSTANCE = new Webcam();

    public static int[] GOAL_TAG_IDS    = new int[]{7, 8, 9, 10, 20, 24};
    public static int[] OBELISK_TAG_IDS = new int[]{21, 22, 23};

    private static final String CAMERA_NAME = "Webcam";

    public  int soleTagID = 0;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    public AprilTagDetection bestGoalDetection;
    private AprilTagDetection bestObeliskDetection;
    private List<AprilTagDetection> lastDetections = new ArrayList<>();

    private Webcam() { }

    @Override
    public void initialize() {
        HardwareMap hw = OpModeData.hardwareMap;

        // 1) Start with the DECODE (current season) tag library
        // This is what enables metadata + pose for official tags. :contentReference[oaicite:4]{index=4}
        AprilTagLibrary.Builder libBuilder = new AprilTagLibrary.Builder();
        libBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // 2) If you are using ANY tags that are not in the DECODE library
        // (ex: printed “test” tags 7/8/9/10/100/101/102), you MUST add them with correct size,
        // otherwise ftcPose will remain null. :contentReference[oaicite:5]{index=5}
        //


        // IMPORTANT: Tag size must be the black square’s side length (in your chosen unit).
        // Replace the sizes below with your real measured values.

        libBuilder.addTag(7,   "5 Inch Test Blue", 5.0, DistanceUnit.INCH);
        libBuilder.addTag(8,   "2 Inch Test Blue", 2.0, DistanceUnit.INCH);
        libBuilder.addTag(9,   "5 Inch Test Red",  5.0, DistanceUnit.INCH);
        libBuilder.addTag(10,  "2 Inch Test Red",  2.0, DistanceUnit.INCH);


        //  libBuilder.addTag(100, "Obelisk Motif 1", 2.0, DistanceUnit.INCH);
      //  libBuilder.addTag(101, "Obelisk Motif 2", 2.0, DistanceUnit.INCH);
      //  libBuilder.addTag(102, "Obelisk Motif 3", 2.0, DistanceUnit.INCH);

        AprilTagLibrary tagLibrary = libBuilder.build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, CAMERA_NAME))
                // Recommended common resolution; helps consistency with common webcam calibrations
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
               // .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }


    @Override
    public void periodic() {
        if (aprilTag == null) return;

        lastDetections = aprilTag.getDetections();

        if (soleTagID != 0) {
            bestGoalDetection = getDetectionById(soleTagID);
            bestObeliskDetection = null;
        } else {
            bestGoalDetection = pickBestByIdSet(lastDetections, GOAL_TAG_IDS);
            bestObeliskDetection = pickBestByIdSet(lastDetections, OBELISK_TAG_IDS);
        }
    }

    public  void setSoleTagID(int GoalID) {
        soleTagID = GoalID;
    }

    public VisionPortal getVisionPortal() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
        aprilTag = null;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("=== WEBCAM / APRILTAGS ===","");
        telemetry.addData("Cam", CAMERA_NAME);
        telemetry.addData("Detections", lastDetections == null ? 0 : lastDetections.size());

        // Show every detection (useful when debugging library/pose issues)
        if (lastDetections != null) {
            for (AprilTagDetection d : lastDetections) {
                telemetry.addData("Tag", "id=%d  meta=%s  ftcPose=%s",
                        d.id,
                        (d.metadata != null ? "Y" : "N"),
                        (d.ftcPose != null ? "Y" : "N"));
            }
        }

        if (bestGoalDetection != null) {
            telemetry.addData("-- Goal --","");
            telemetry.addData("ID", bestGoalDetection.id);
            telemetry.addData("Name", goalNameFromId(bestGoalDetection.id));
            if (bestGoalDetection.ftcPose != null) {
                telemetry.addData("Range (in)", bestGoalDetection.ftcPose.range);
                telemetry.addData("Bearing (deg)", bestGoalDetection.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", bestGoalDetection.ftcPose.yaw);
            } else {
                telemetry.addData("Goal pose is NULL (tag missing from library / size unknown).","");
            }
        } else {
            telemetry.addData("Goal: none","");
        }

        if (bestObeliskDetection != null) {
            telemetry.addData("-- Obelisk --","");
            telemetry.addData("ID", bestObeliskDetection.id);
            telemetry.addData("Auto Motif", autoMotifFromId(bestObeliskDetection.id));
        } else {
            telemetry.addData("Obelisk: none","");
        }
    }

    public String goalNameFromId(int tagId) {
        if (tagId == 7)  return "5 Inch Test Blue";
        if (tagId == 8)  return "2 Inch Test Blue";
        if (tagId == 9)  return "5 Inch Test Red";
        if (tagId == 10) return "2 Inch Test Red";
        if (tagId == 20) return "Goal Blue";
        if (tagId == 24) return "Goal Red";
        return "Unknown Goal";
    }

    public String autoMotifFromId(int tagId) {
        if (tagId == 21) return "GPP";
        if (tagId == 22) return "PGP";
        if (tagId == 23) return "PPG";
        return "Unknown Motif";
    }

    private static AprilTagDetection pickBestByIdSet(List<AprilTagDetection> detections, int[] ids) {
        if (detections == null || detections.isEmpty() || ids == null || ids.length == 0) return null;

        AprilTagDetection best = null;
        double bestRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (d == null) continue;
            if (!contains(ids, d.id)) continue;

            // If ftcPose is null, it means no metadata => no range estimate.
            if (d.ftcPose == null) continue;

            if (d.ftcPose.range < bestRange) {
                bestRange = d.ftcPose.range;
                best = d;
            }
        }
        return best;
    }

    public AprilTagDetection getDetectionById(int tagId) {
        if (lastDetections == null) return null;

        AprilTagDetection best = null;
        double bestRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : lastDetections) {
            if (d.id == tagId && d.ftcPose != null) {
                if (d.ftcPose.range < bestRange) {
                    bestRange = d.ftcPose.range;
                    best = d;
                }
            }
        }
        return best;
    }



    private static boolean contains(int[] arr, int value) {
        for (int x : arr) if (x == value) return true;
        return false;
    }


    public double getRange() {
        return bestGoalDetection.ftcPose.range;
    }

    public  boolean seesTag() {
        if (bestGoalDetection == null) {
            return false;
        }
        return   bestGoalDetection.ftcPose != null;
    }

}
