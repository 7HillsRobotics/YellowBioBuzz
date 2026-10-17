package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Class to control the shooter flywheel and hood for the BioBuzz season.
 * Integrates with Vision to adjust shot parameters based on distance to targets.
 */
@SuppressWarnings("unused")
public class Shooter {

    private final DcMotorEx flywheelMotor;
    private final Servo hoodServo;
    private final Vision vision;

    // Control constants
    public static double SHOOT_POWER = 0.9;
    public static final double MAX_POWER = 1.0;

    public Shooter(HardwareMap hardwareMap, Vision vision) {
        this.flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        this.vision = vision;
    }

    /**
     * Updates the shooter state, including automated hood adjustment based on AprilTag distance.
     *
     * @param targetId The ID of the AprilTag to track for distance estimation.
     */
    public void update(int targetId) {
        List<AprilTagDetection> detections = vision.getDetections();
        AprilTagDetection target = findDetectionById(detections, targetId);

        if (target != null && target.ftcPose != null) {
            double range = target.ftcPose.range;

            // Automated hood logic - finisher will need to calibrate this mapping
            double calculatedHoodPos = calculateHoodPosition(range);
            setHoodPosition(calculatedHoodPos);
        }
    }

    /**
     * Calculates the required hood position for a given range to target.
     * To be implemented with a lookup table or mathematical model by the finisher.
     */
    private double calculateHoodPosition(double range) {
        // TODO: Implement distance-to-hood mapping (e.g., linear interpolation or lookup table)
        return 0.5;
    }

    /**
     * Runs the flywheel at the specified shooting power.
     */
    public void shoot() {
        setPower(SHOOT_POWER);
    }

    /**
     * Sets the power of the flywheel motor.
     */
    public void setPower(double power) {
        flywheelMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Sets the position of the hood servo.
     */
    public void setHoodPosition(double position) {
        hoodServo.setPosition(Range.clip(position, 0.0, 1.0));
    }

    /**
     * Stops the shooter motor.
     */
    public void stop() {
        flywheelMotor.setPower(0);
    }

    /**
     * Helper to find a specific tag by ID in the list of detections.
     */
    private @Nullable AprilTagDetection findDetectionById(List<AprilTagDetection> detections, int id) {
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                if (((AprilTagSingleDetection) detection).id == id) {
                    return detection;
                }
            }
        }
        return null;
    }
}
