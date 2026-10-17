package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

/**
 * Vision subsystem for the BioBuzz (2026-2027) season.
 * <p>
 * This subsystem handles:
 * 1. AprilTag detection (Specifically Clusters for aiming at moving Cell openings).
 * 2. Color Blob tracking for Pollen (Yellow) and Nectar (Red and Blue alliance-specific).
 * 3. Shooter intake ball detection via color sensor.
 */
@SuppressWarnings("unused")
public class Vision {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final ColorBlobLocatorProcessor pollenLocator;
    private final ColorBlobLocatorProcessor redNectarLocator;
    private final ColorBlobLocatorProcessor blueNectarLocator;

    public Vision(HardwareMap hardwareMap) {
        // 1. Initialize AprilTag Processor
        // BioBuzz specific AprilTag library is returned by default in SDK 12.0
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        // 2. Initialize Pollen Locator (Yellow blobs)
        pollenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                // ROI defined using normalized +/- 1.0 coordinates: left, top, right, bottom
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .build();

        // 3. Initialize Red Nectar Locator
        redNectarLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // 4. Initialize Blue Nectar Locator
        blueNectarLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // 5. Create the Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag, pollenLocator, redNectarLocator, blueNectarLocator)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * Retrieves current AprilTag detections. 
     * BioBuzz Note: Check for instanceof AprilTagClusterDetection for aiming at Cells.
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Logic stub for processing aiming and navigation targets.
     */
    public void updateTargets() {
        List<AprilTagDetection> detections = getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                // TODO: Calculate turret aiming logic using cluster origin (Cell center)
            } else if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                // TODO: Handle navigation using standalone tags
            }
        }
    }

    /**
     * Returns detected Pollen (Yellow) blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getPollenBlobs() {
        return pollenLocator.getBlobs();
    }

    /**
     * Returns detected Red Nectar blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getRedNectarBlobs() {
        return redNectarLocator.getBlobs();
    }

    /**
     * Returns detected Blue Nectar blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getBlueNectarBlobs() {
        return blueNectarLocator.getBlobs();
    }

    /**
     * Toggles specific processors to save CPU resources.
     */
    public void setProcessorsEnabled(boolean tags, boolean pollen, boolean redNectar, boolean blueNectar) {
        visionPortal.setProcessorEnabled(aprilTag, tags);
        visionPortal.setProcessorEnabled(pollenLocator, pollen);
        visionPortal.setProcessorEnabled(redNectarLocator, redNectar);
        visionPortal.setProcessorEnabled(blueNectarLocator, blueNectar);
    }

    /**
     * Releases camera resources.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
