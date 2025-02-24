package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * The Gripper class manages all of the servo components on the gripper.
 * It provides preset methods such as rotation, orientation, pickupPosition, etc.
 * Note that users of this class never directly control individual servo positions.
 */

public class Gripper {

    // Configurable positions or direction for gripper
    public double rotationPosition = 0.9;
    public double orientationPosition = 0.1;
    public double inOutTakeDirection = 0;

    // Servo hardware references
    private final Servo GripperRotation;
    private final Servo GripperOrientation;
    private final CRServo GripperInOutTake;

    // Override flag for sensor-based methods
    public boolean override = false;
    // Timer
    ElapsedTime holdTimer;

    /**
     * Constructs the EndEffector subsystem using the provided hardware map.
     * @param hardwareMap The hardware map from the op mode.
     */
    public Gripper(HardwareMap hardwareMap) {
        // Map servos
        GripperOrientation = hardwareMap.get(Servo.class, "GripperOrientation");
        GripperRotation = hardwareMap.get(Servo.class, "GripperRotation");
        GripperInOutTake = hardwareMap.get(CRServo.class, "GripperInOutTake");
    }

    /**
     * Sets the rotation servo to a specified position.
     * The position is clipped to the range [0.38, 0.7].
     * @param position The desired rotation position.
     */
    public void setRotationPosition(double position) {
        position = Range.clip(position, 0, 1);
        GripperRotation.setPosition(position);
        rotationPosition = position;
    }

    /**
     * Sets the orientation servo to a specified position.
     * The position is clipped to the range [0.38, 0.7].
     * @param position The desired orientation position.
     */
    public void setOrientationPosition(double position) {
        position = Range.clip(position, 0, 1);
        GripperOrientation.setPosition(position);
        orientationPosition = position;
    }

    /**
     * Sets the intake/outtake servo to a specified direction.
     * @param power The desired intake/outtake direction power.
     */
    public void setInOutTakeDirection(double power) {
        GripperInOutTake.setPower(power);
        inOutTakeDirection = power;
    }

    /**
     * Gets the current rotation servo current position.
     * @return The rotation servo position.
     */

    public double getRotationPosition() {
        return GripperRotation.getPosition();
    }

    /**
     * Gets the current orientation servo current position.
     * @return The orientation servo position.
     */
    public double getOrientationPosition() {
        return GripperOrientation.getPosition();
    }

    // --- Preset Position Methods ---

    /**
     * Initializes the gripper to a default starting state.
     */
    public void init() {
        setGripperHomePosition();
    }

    /**
     * Sets the gripper to a home position.
     * first value for rotation, second value for orientation
     */
    public void setGripperHomePosition() {
        GripperRotation.setPosition(1);
        GripperOrientation.setPosition(0.1);

    }

    /**
     * Sets the gripper for travel position
     * first value for rotation, second value for orientation
     */
    public void setGripperTravelPosition() {
        GripperRotation.setPosition(0.9);
        GripperOrientation.setPosition(0.1);
        //setPositions(0.9, 0.1);
    }

    /**
     * Sets the gripper to the bucket scoring position.
     * first value for rotation, second value for orientation
     */
    public void setBucketScorePosition() {
        GripperRotation.setPosition(0.65);
        GripperOrientation.setPosition(0.1);
        //setPositions(0.65, 0.1);

    }

    /**
     * Sets the gripper to the floor sample pickup position 1.
     */
    public void setPrePickupPosition(double orientPos) {
        // variable orientation based on sample position
        setPositions(0, orientPos);
    }

    /**
     * Sets the gripper for pre specimen placement
     */
    public void setPreSpecimenPlacement() {
        GripperRotation.setPosition(0.7);
        GripperOrientation.setPosition(0.5);
    }

    /**
     * Sets the gripper for pushing specimen onto the bar
     * first value for rotation, second value for orientation
     */
    public void setPushSpecimemOnBar() {
        GripperRotation.setPosition(0.7);
        GripperOrientation.setPosition(0.5);
        //setPositions(0.7, 0.5);
    }

    /**
     * Sets the gripper for wall specimen pickup
     * first value for rotation, second value for orientation
     */
    public void setWallIntakePosition() {
        GripperRotation.setPosition(0.39);
        GripperOrientation.setPosition(0.5);
        //setPositions(0.39, 0.5);
    }

    /**
     * Sets the gripper for wall specimen pickup
     * first value for rotation, second value for orientation
     */
    public void setWallSpecRemovalPosition() {
        GripperRotation.setPosition(0.5);
        GripperOrientation.setPosition(0.5);
        //setPositions(0.5, 0.5);
    }

    /**
     * Spin gripper fingers for intake.
     */
    public void gripperIntake() {
        GripperInOutTake.setPower(-1);
    }

    /**
     * Spin gripper fingers for output.
     */
    public void gripperOuttake() {
        GripperInOutTake.setPower(1);
    }

    /**
     * Dump the sample into the basket.
     */
    private void dumpSample(double dumpTime) {
        holdTimer.reset();
        while (holdTimer.seconds() < dumpTime) {
            GripperInOutTake.setPower(-1);
        }
        GripperInOutTake.setPower(0);
    }

    /**
     * Sets the positions of the rotation and orientation servos.
     *
     * @param rotationPos   Position for the arm servos.
     * @param orientationPos Position for the pivot servo.
     */
    public void setPositions(double rotationPos, double orientationPos) {
        setRotationPosition(rotationPos);
        setOrientationPosition(orientationPos);
    }

}
