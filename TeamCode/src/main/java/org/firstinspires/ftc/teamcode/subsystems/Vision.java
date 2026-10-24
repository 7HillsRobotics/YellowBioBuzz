package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.Collections;
import java.util.List;

/**
 * Class to detect Cell targets, Pollen, and Nectar for the BioBuzz season.
 */
@SuppressWarnings("unused")
public class Vision {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final ColorBlobLocatorProcessor pollenLocator;
    private final ColorBlobLocatorProcessor redNectarLocator;
    private final ColorBlobLocatorProcessor blueNectarLocator;
    private boolean closed = false;

    // Control constants - tune blob limits using the camera preview
    public static double MIN_BLOB_AREA = 50;
    public static double MAX_BLOB_AREA = 20000;

    public Vision(HardwareMap hardwareMap) {
        // Initialize AprilTag detection
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .build();

        // Initialize color detection
        pollenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                // ROI defined using normalized +/- 1.0 coordinates: left, top, right, bottom
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))
                .build();

        redNectarLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        blueNectarLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .build();

        // Open the camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag, pollenLocator, redNectarLocator, blueNectarLocator)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    /**
     * Returns current AprilTag detections while the processor is running.
     */
    public List<AprilTagDetection> getDetections() {
        if (closed || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING ||
                !visionPortal.getProcessorEnabled(aprilTag)) {
            return Collections.emptyList();
        }
        return aprilTag.getDetections();
    }

    /**
     * Finds a single tag with a valid pose by ID.
     */
    public @Nullable AprilTagSingleDetection getTag(int targetId) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection instanceof AprilTagSingleDetection && hasPose(detection)) {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                if (single.id == targetId) return single;
            }
        }
        return null;
    }

    /**
     * Finds a Cell cluster with a valid pose by its SDK name.
     */
    public @Nullable AprilTagClusterDetection getCluster(String clusterName) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection instanceof AprilTagClusterDetection && hasPose(detection)) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && cluster.metadata.shortName.equals(clusterName)) {
                    return cluster;
                }
            }
        }
        return null;
    }

    private boolean hasPose(AprilTagDetection detection) {
        return detection.ftcPose != null && Double.isFinite(detection.ftcPose.range) &&
                detection.ftcPose.range > 0 && Double.isFinite(detection.ftcPose.bearing);
    }

    /**
     * Returns detected Pollen (Yellow) blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getPollenBlobs() {
        return getBlobs(pollenLocator);
    }

    /**
     * Returns detected Red Nectar blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getRedNectarBlobs() {
        return getBlobs(redNectarLocator);
    }

    /**
     * Returns detected Blue Nectar blobs.
     */
    public List<ColorBlobLocatorProcessor.Blob> getBlueNectarBlobs() {
        return getBlobs(blueNectarLocator);
    }

    private List<ColorBlobLocatorProcessor.Blob> getBlobs(ColorBlobLocatorProcessor locator) {
        if (closed || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING ||
                !visionPortal.getProcessorEnabled(locator)) {
            return Collections.emptyList();
        }
        List<ColorBlobLocatorProcessor.Blob> blobs = locator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, MIN_BLOB_AREA, MAX_BLOB_AREA, blobs);
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
        return blobs;
    }

    /**
     * Returns the camera state for driver telemetry.
     */
    public VisionPortal.CameraState getCameraState() {
        return visionPortal.getCameraState();
    }

    /**
     * Toggles specific processors to save CPU resources.
     */
    public void setProcessorsEnabled(boolean tags, boolean pollen, boolean redNectar, boolean blueNectar) {
        if (closed) return;
        visionPortal.setProcessorEnabled(aprilTag, tags);
        visionPortal.setProcessorEnabled(pollenLocator, pollen);
        visionPortal.setProcessorEnabled(redNectarLocator, redNectar);
        visionPortal.setProcessorEnabled(blueNectarLocator, blueNectar);
    }

    /**
     * Releases camera resources.
     */
    public void close() {
        if (!closed) {
            closed = true;
            visionPortal.close();
        }
    }
}
