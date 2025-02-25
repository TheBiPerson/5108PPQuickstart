package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The LiftArmSlide controls the chain lift arm and the slide extension.
 * It provides preset methods such as arm rotation or slide location.
 * In addition, the lift arm and slide home method is included here.
 */

public class LiftArmSlide {
    public DcMotorEx ChainLiftMotor;
    public DcMotorEx SlideMotor;
    public TouchSensor SlideHomeMagTouch;
    public TouchSensor ChainLiftHomeTouch;

    public double slideTarget = 0;
    public double liftArmTarget = 0;
    public boolean slidesReached;
    public boolean liftArmReached;
    // Between retracted and extended
    public boolean slidesRetracted;
    public int slidePos;
    public double maxSlide = 1300;
    public int liftArmPos;
    private boolean debug = false;

    public static double CHAIN_ARM_POWER = 0.5;
    public static double SLIDE_POWER = 1.0;

    public LiftArmSlide(HardwareMap hardwareMap, Telemetry telemetry) {

        ChainLiftHomeTouch = hardwareMap.get(TouchSensor.class, "ChainLiftHomeTouch");
        SlideHomeMagTouch = hardwareMap.get(TouchSensor.class, "SlideHomeMagTouch");

        ChainLiftMotor = hardwareMap.get(DcMotorEx.class, "ChainLiftMotor");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");

        ChainLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ChainLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        SlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF control is essential to stop motor oscillation
        ChainLiftMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        ChainLiftMotor.setPositionPIDFCoefficients(10);
        SlideMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        SlideMotor.setPositionPIDFCoefficients(10);
    }

    /**
     * Move the ChainLiftArm to a target position.
     * Hold the LiftArm in place while performing some action.
     */
    public void rotateLiftArm(int armTarget, double power) {
        ChainLiftMotor.setTargetPosition(armTarget);
        ChainLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ChainLiftMotor.setPower(power);
        //sleep(10);
    }

    /**
     * Extend or retract the slide to a position
     */
    public void moveSlideArm(int slideTarget) {
        SlideMotor.setTargetPosition(slideTarget);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Retract the slide and lower chain drive to home position. To help eliminate possible mechanism
     * conflicts, raise the arm slightly before moving the gripper and retracting the slide.
     */
    public void homeChainLiftArm() {
        // Raise the chain drive assembly slightly to eliminate mechanical conflicts.
        ChainLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ChainLiftMotor.setTargetPosition(250);
        ChainLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ChainLiftMotor.setPower(0.2);
        // Retract the slide to the home position.
        while (!SlideHomeMagTouch.isPressed()) {
            if (SlideHomeMagTouch.isPressed()) {
                SlideMotor.setPower(0);
                slidePos = 0;
                telemetry.addData("Slide", "is retracted");
                telemetry.update();
                break;
            }
            SlideMotor.setPower(-0.3);
        }
        // At this point, the slide should be home, let's finalize slide.
        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setTargetPosition(slidePos);
        SlideMotor.setPower(SLIDE_POWER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // The chain lift arm needs to be placed in the home position.
        while (!ChainLiftHomeTouch.isPressed()) {
            if (ChainLiftHomeTouch.isPressed()) {
                ChainLiftMotor.setPower(0);
                liftArmPos = 0;
                telemetry.addData("Arm", "is retracted");
                telemetry.update();
                break;
            }
            ChainLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ChainLiftMotor.setPower(-0.4);
        }
        ChainLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ChainLiftMotor.setTargetPosition(liftArmPos);
        ChainLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ChainLiftMotor.setPower(CHAIN_ARM_POWER);
        if (debug) {
            telemetry.addData("current chain arm position", ChainLiftMotor.getCurrentPosition());
            telemetry.addData("current slide position", SlideMotor.getCurrentPosition());
            telemetry.update();
            //sleep(3000);
        }
    }


    /**
     * Get the current position of the slide motor
     * @return current slide motor position in ticks
     */
    public int getSlidePos() {
        return SlideMotor.getCurrentPosition();
    }

    /**
     * Get the current position of the chain lift arm motor
     * @return the current position of the lift arm in ticks
     */
    public int getLiftArmPos() {
        return ChainLiftMotor.getCurrentPosition();
    }

    // --- Preset Position Methods ---

    // Set the lift arm and slide to home position
    public void setLiftArmSlideHomePos() {
        rotateLiftArm(10, CHAIN_ARM_POWER);
        moveSlideArm(10);
    }

    // Set the lift arm and slide to travel position
    public void setLiftArmSlideTravelPos() {
        rotateLiftArm(175, CHAIN_ARM_POWER);
        moveSlideArm(100);
    }

    // Set the lift arm and slide to prescore specimen position
    public void setLiftArmSlidePreSpecScorePos() {
        rotateLiftArm(475, CHAIN_ARM_POWER);
        moveSlideArm(1250);
    }

    // Set the slide to push specimen onto rung position
    public void setSlideToPushSpecPos() {
        moveSlideArm(1750);
    }

    // Set lift arm and slide for wall specimen pickup
    public void setLiftArmSlidePreWallPickup() {
        rotateLiftArm(300, CHAIN_ARM_POWER);
        moveSlideArm(750);
    }

    // Set the slide to grab specimen from wall
    public void setSlideForWallPickup() {
        moveSlideArm(1150);
    }

    // Set lift arm and slide after grabbing specimen from wall
    public void setRemoveWallSpecimen() {
        rotateLiftArm(500, CHAIN_ARM_POWER);
        //sleep(100);
        moveSlideArm(450);
    }
}
