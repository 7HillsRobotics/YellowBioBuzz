package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Class to control intake functionality for the BioBuzz season.
 * Currently configured to simply take balls in.
 * Needs to be configured with all internals as well.
 */
@SuppressWarnings("unused")
public class Intake {

    private final DcMotorEx intakeMotor;

    // Control constants
    public static double INTAKE_POWER = 0.8;
    public static final double MAX_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    /**
     * Updates the intake state. 
     * Finisher can add logic to use vision for automated collection.
     */
    public void update(boolean run) {
        if (run) {
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
     * Sets the power of the intake motor.
     */
    public void setPower(double power) {
        intakeMotor.setPower(Range.clip(power, -MAX_POWER, MAX_POWER));
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }
}
