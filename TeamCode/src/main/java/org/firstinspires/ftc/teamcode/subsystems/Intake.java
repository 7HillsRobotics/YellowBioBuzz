package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Class to control intake functionality for the BioBuzz season.
 * Runs the intake forward to collect balls or backward to clear them.
 */
@SuppressWarnings("unused")
public class Intake {

    private final DcMotorEx intakeMotor;

    // Control constants
    public static double INTAKE_POWER = 0.8;
    public static final double MAX_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        stop();
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Runs the intake while requested, otherwise stops it.
     */
    public void update(boolean run) {
        update(run, false);
    }

    /**
     * Updates the intake state. Reverse takes priority over collection.
     */
    public void update(boolean run, boolean reverse) {
        if (reverse) {
            outtake();
        } else if (run) {
            intake();
        } else {
            stop();
        }
    }

    /**
     * Runs the intake motor at a constant power to collect balls.
     */
    public void intake() {
        setPower(INTAKE_POWER);
    }

    /**
     * Runs the intake backward to release balls or clear a jam.
     */
    public void outtake() {
        setPower(-INTAKE_POWER);
    }

    /**
     * Sets the power of the intake motor.
     */
    public void setPower(double power) {
        if (!Double.isFinite(power)) power = 0;
        intakeMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}
