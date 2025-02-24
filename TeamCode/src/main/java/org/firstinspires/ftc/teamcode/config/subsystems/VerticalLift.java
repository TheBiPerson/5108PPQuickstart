package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class VerticalLift {

    private CRServo ClimbVerticalLift;
    private TouchSensor ClimbHomeTouch;
    private TouchSensor ClimbExtendTouch;

    public VerticalLift(HardwareMap hardwareMap) {
        ClimbVerticalLift = hardwareMap.get(CRServo.class, "ClimbVerticalLift");
        ClimbHomeTouch = hardwareMap.get(TouchSensor.class, "ClimbHomeTouch");
        ClimbExtendTouch = hardwareMap.get(TouchSensor.class, "ClimbExtendTouch");

    }

    /**
     * Lower Climbing Lift function to ensure lift is in home position at start of autonomous
     */
    public void lowerClimbingLift() {
        // Climbing Lift MUST be placed in fully retracted down position.
        if (!ClimbHomeTouch.isPressed()) {
            //Negative power lowers the vertical lift
            ClimbVerticalLift.setPower(-0.5);
            while (!ClimbHomeTouch.isPressed()) {
                ClimbVerticalLift.setPower(-0.5);
                if (ClimbHomeTouch.isPressed()) {
                    ClimbVerticalLift.setPower(0);
                    break;
                }
            }
        } else if (ClimbHomeTouch.isPressed()) {
            ClimbVerticalLift.setPower(0);
        }
    }

    /**
     * Raise climbing Lift to enable touching the low rung for Level 1 Parking in autonomous.
     */
    public void raiseClimbingLift() {
        // Climbing Lift needs to be fully extended to reach lower rung.
        if (!ClimbExtendTouch.isPressed()) {
            // Positive power raises the Lift Arm
            ClimbVerticalLift.setPower(0.9);
            while (!ClimbExtendTouch.isPressed()) {
                ClimbVerticalLift.setPower(0.9);
                if (ClimbExtendTouch.isPressed()) {
                    ClimbVerticalLift.setPower(0);
                    break;
                }
            }
        } else if (ClimbExtendTouch.isPressed()) {
            ClimbVerticalLift.setPower(0);
        }
    }
}
