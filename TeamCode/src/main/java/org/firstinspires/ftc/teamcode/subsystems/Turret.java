package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Class to control turret aiming toward AprilTags using a continuously rotating servo.
 * Integrates with the Vision subsystem to retrieve tracking data.
 */
@SuppressWarnings("unused")
public class Turret {

    private final CRServo turretServo;
    private final Vision vision;

    // Control constants - to be tuned by the developer
    public static double Kp = 0.02;
    public static final double MAX_POWER = 1.0;
    public static final double DEADZONE_DEG = 1.0;

    public Turret(HardwareMap hardwareMap, Vision vision) {
        this.turretServo = hardwareMap.get(CRServo.class, "turretServo");
        this.vision = vision;
        stop();
    }

    /**
     * Updates the turret power to aim at a specific AprilTag ID using detections from Vision.
     *
     * @param targetId The ID of the AprilTag to track.
     */
    public void update(int targetId) {
        aimAt(vision.getTag(targetId));
    }

    /**
     * Aims at the center of the named Cell cluster.
     */
    public void update(String clusterName) {
        aimAt(vision.getCluster(clusterName));
    }

    private void aimAt(@Nullable AprilTagDetection target) {
        if (target != null) {
            // bearing is the horizontal angle to the tag in degrees
            double bearing = target.ftcPose.bearing;

            if (Math.abs(bearing) > DEADZONE_DEG) {
                // Simple proportional loop to rotate turret toward the tag
                double power = bearing * Kp;
                setPower(power);
            } else {
                stop();
            }
        } else {
            // No target found or no pose data
            stop();
        }
    }

    /**
     * Sets the power of the turret servo.
     */
    public void setPower(double power) {
        if (!Double.isFinite(power)) power = 0;
        turretServo.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Stops the turret.
     */
    public void stop() {
        turretServo.setPower(0);
    }
}
