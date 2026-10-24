package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Class to control the shooter flywheel and hood for the BioBuzz season.
 * Keeps manual hood control until distance-based settings are measured.
 */
@SuppressWarnings("unused")
public class Shooter {

    private final DcMotorEx flywheelMotor;
    private final Servo hoodServo;
    private final Vision vision;

    // Control constants - tune these on the robot before shooting
    public static double SHOOT_POWER = 0.9;
    public static double HOOD_POSITION = 0.5;
    public static final double MAX_POWER = 1.0;

    public Shooter(HardwareMap hardwareMap, Vision vision) {
        this.flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        this.hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        this.vision = vision;
        stop();
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        setHoodPosition(HOOD_POSITION);
    }

    /**
     * Runs the flywheel while requested, otherwise stops it.
     */
    public void update(boolean run) {
        if (run) {
            shoot();
        } else {
            stop();
        }
    }

    /**
     * Updates the hood when the requested AprilTag has a pose.
     * Keeps the current hood position when no target is found.
     *
     * @param targetId The ID of the AprilTag to track.
     */
    public void update(int targetId) {
        updateHood(vision.getTag(targetId));
    }

    /**
     * Updates the hood using the center of the named Cell cluster.
     */
    public void update(String clusterName) {
        updateHood(vision.getCluster(clusterName));
    }

    private void updateHood(@Nullable AprilTagDetection target) {
        if (target != null) {
            setHoodPosition(calculateHoodPosition(target.ftcPose.range));
        }
    }

    private double calculateHoodPosition(double range) {
        // shortcut: distance settings are not measured, replace this return after calibration.
        return hoodServo.getPosition();
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
        if (!Double.isFinite(power)) power = 0;
        flywheelMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Sets the position of the hood servo.
     */
    public void setHoodPosition(double position) {
        if (!Double.isFinite(position)) return;
        hoodServo.setPosition(Range.clip(position, 0.0, 1.0));
    }

    /**
     * Returns the last commanded hood position.
     */
    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /**
     * Stops the shooter motor.
     */
    public void stop() {
        flywheelMotor.setPower(0);
    }
}
