package org.firstinspires.ftc.teamcode.mechanisms;

import android.hardware.HardwareBuffer;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;
    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

    }
    public void update(){
        detectedTags = aprilTagProcessor.getDetections();
    }
    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }
    public void displayDetectionTelemetry(AprilTagDetection tag) {
        if (tag == null) {
            telemetry.addLine("❌ Tag not detected");
            telemetry.update();
            return;
        }

        telemetry.addLine(String.format("🆔 Tag ID: %d", tag.id));
        if (tag.metadata != null) {
            telemetry.addLine(String.format("Name: %s", tag.metadata.name));

            telemetry.addLine("\n📍 Position (cm):");
            telemetry.addLine(String.format("   X: %.1f   Y: %.1f   Z: %.1f",
                    tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));

            telemetry.addLine("\n📏 Distance and Direction:");
            telemetry.addLine(String.format("   Range: %.1f cm", tag.ftcPose.range));
            telemetry.addLine(String.format("   Bearing: %.1f°   Elevation: %.1f°",
                    tag.ftcPose.bearing, tag.ftcPose.elevation));

            telemetry.addLine("\n🌀 Orientation (deg):");
            telemetry.addLine(String.format("   Pitch: %.1f   Roll: %.1f   Yaw: %.1f",
                    tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
        } else {
            telemetry.addLine("⚠️  No metadata available");
            telemetry.addLine(String.format("Center (px):  X=%.0f  Y=%.0f",
                    tag.center.x, tag.center.y));
        }

        telemetry.update();
    }

    public AprilTagDetection getTagBySpecificId(int id){
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }
    public void stop(){
        if (visionPortal != null){
            visionPortal.close();
        }
    }



}
