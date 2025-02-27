package org.firstinspires.ftc.teamcode.opmode.Auto;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BESA_Five_Spec_Auto", group = "auto")
public class BESA_Five_Spec_Auto extends OpMode {

//*************************** Begin LiftArm and Slide ****************************************
    /**
     * The initLiftArmSlide controls the chain lift arm and the slide extension.
     * It provides preset methods such as arm rotation or slide location and
     * other associated lift arm and slide variables, declarations, and methods..
     * In addition, the lift arm and slide home method is included here.
     */
    private DcMotorEx ChainLiftMotor;
    private DcMotorEx SlideMotor;
    private TouchSensor SlideHomeMagTouch;
    private TouchSensor ChainLiftHomeTouch;

    private double slideTarget = 0;
    private double liftArmTarget = 0;
    // Between retracted and extended
    private int slidePos;
    private double maxSlide = 1300;
    private int liftArmPos;
    private boolean debug = false;

    private double CHAIN_ARM_POWER = 1.0;
    private double SLIDE_POWER = 1.0;


    public void initLiftArmSlide() {
        ChainLiftMotor = hardwareMap.get(DcMotorEx.class, "ChainLiftMotor");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        SlideHomeMagTouch = hardwareMap.get(TouchSensor.class, "SlideHomeMagTouch");
        ChainLiftHomeTouch = hardwareMap.get(TouchSensor.class, "ChainLiftHomeTouch");

        ChainLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ChainLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        SlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF control is essential to stop motor oscillation
        ChainLiftMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        ChainLiftMotor.setPositionPIDFCoefficients(13.5);
        SlideMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        SlideMotor.setPositionPIDFCoefficients(13.5);
    }

    // Move the ChainLiftArm to a target position.
    private void rotateLiftArm(int armTarget, double power) {
        ChainLiftMotor.setTargetPosition(armTarget);
        //ChainLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ChainLiftMotor.setPower(power);
    }

    // Extend or retract the slide to a position
    private void moveSlideArm(int slideTarget) {
        SlideMotor.setTargetPosition(slideTarget);
       //SlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Retract the slide and lower chain drive to home position. To help eliminate possible mechanism
     * conflicts, raise the arm slightly before moving the gripper and retracting the slide.
     */
    private void homeChainLiftArm() {
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
                //telemetry.addData("Slide", "is retracted");
                //telemetry.update();
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
                //telemetry.addData("Arm", "is retracted");
                //telemetry.update();
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
            //telemetry.addData("current chain arm position", ChainLiftMotor.getCurrentPosition());
            //telemetry.addData("current slide position", SlideMotor.getCurrentPosition());
            //telemetry.update();
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

    // ------------- initLiftArmSlide Preset Position Methods -------------------

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
        rotateLiftArm(555, CHAIN_ARM_POWER);
        moveSlideArm(1000);
    }

    // Set the slide to push specimen onto rung position
    public void setSlideToPushSpecPos() {
        moveSlideArm(1350);
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

//*************************** End LiftArm and Slide ******************************************
//
//
//****************************** Begin Gripper ***********************************************
    /**
     * The Gripper methods manage all of the servo components on the gripper.
     * It provides preset methods such as rotation, orientation, pickupPosition, etc.
     * Note that users of this class never directly control individual servo positions.
     */
    // Configurable positions or direction for gripper
    public double rotationPosition = 0.9;
    public double orientationPosition = 0.1;
    public double inOutTakeDirection = 0;

    // Servo hardware references
    private Servo GripperRotation;
    private Servo GripperOrientation;
    private CRServo GripperInOutTake;

    public void initGripper() {
        GripperRotation = hardwareMap.get(Servo.class, "GripperRotation");
        GripperOrientation = hardwareMap.get(Servo.class, "GripperOrientation");
        GripperInOutTake = hardwareMap.get(CRServo.class, "GripperInOutTake");
    }
    // Timer
    ElapsedTime holdTimer;
    /**
     * Constructs the Gripper subsystem using the provided hardware map.
     * @param hardwareMap The hardware map from the op mode.
     */

    /**
     * Sets the Rotation servo to a specified position.
     * @param position The desired rotation position.
     */
    private void setRotationPosition(double position) {
        GripperRotation.setPosition(position);
        rotationPosition = position;
    }

    /**
     * Sets the Orientation servo to a specified position.
     * @param position The desired orientation position.
     */
    private void setOrientationPosition(double position) {
        GripperOrientation.setPosition(position);
        orientationPosition = position;
    }

    /**
     * Sets the intake/outtake servo to a specified (power) direction.
     * @param power The desired intake/outtake direction power.
     */
    private void setInOutTakeDirection(double power) {
        GripperInOutTake.setPower(power);
        inOutTakeDirection = power;
    }

    /**
     * Gets the current rotation servo current position.
     * @return The rotation servo position.
     */
    private double getRotationPosition() {
        return GripperRotation.getPosition();
    }

    /**
     * Gets the current orientation servo current position.
     * @return The orientation servo position.
     */
    private double getOrientationPosition() {
        return GripperOrientation.getPosition();
    }

    // ---------------- Gripper Preset Position Methods ---------------------

    // Sets the gripper to a home position.
    private void setGripperHomePosition() {
        GripperRotation.setPosition(1);
        GripperOrientation.setPosition(0.1);
    }

    // Sets the gripper for travel position
    private void setGripperTravelPosition() {
        GripperRotation.setPosition(0.9);
        GripperOrientation.setPosition(0.1);
    }

    // Sets the gripper for pre specimen placement
    private void setPreSpecimenPlacement() {
        GripperRotation.setPosition(0.7);
        GripperOrientation.setPosition(0.5);
    }



    // Sets the gripper for pushing specimen onto the bar
    private void setPushSpecimenOnBar() {
        GripperRotation.setPosition(0.7);
        GripperOrientation.setPosition(0.5);
    }

    // Sets the gripper for wall specimen pickup
    private void setWallIntakePosition() {
        GripperRotation.setPosition(0.39);
        GripperOrientation.setPosition(0.5);
    }

    // Sets the gripper for wall specimen removal
    private void setWallSpecRemovalPosition() {
        GripperRotation.setPosition(0.5);
        GripperOrientation.setPosition(0.5);
    }

    // Spin gripper fingers for intake.
    private void gripperIntake() {
        GripperInOutTake.setPower(-1);
    }

    // Spin gripper fingers for output.
    private void gripperOuttake() {
        GripperInOutTake.setPower(1);
    }

    //stop spinning fingers
    private void gripperStop() {
        GripperInOutTake.setPower(0);
    }
//********************************* End Gripper **********************************************
//
//
//****************************** Begin Vertical Lift *****************************************
    // All associated Vertical Lift variables, declarations, and methods.

    private CRServo ClimbVerticalLift;
    private TouchSensor ClimbHomeTouch;
    private TouchSensor ClimbExtendTouch;

    public void initClimb() {
        ClimbVerticalLift = hardwareMap.get(CRServo.class, "ClimbVerticalLift");
        ClimbHomeTouch = hardwareMap.get(TouchSensor.class, "ClimbHomeTouch");
        ClimbExtendTouch = hardwareMap.get(TouchSensor.class, "ClimbExtendTouch");
        ClimbVerticalLift.setDirection(DcMotorEx.Direction.REVERSE);
    }

    // Lower Climbing Lift function to ensure lift is in home position at start of autonomous
    private void lowerClimbingLift() {
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

    // Raise climbing Lift to enable touching the low rung for Level 1 Parking in autonomous.
    private void raiseClimbingLift() {
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

//******************************* End Vertical Lift ******************************************
//
//
//************************** Begin Color and Blinkin *****************************************
    // All associated Color and Blinkin variables, declarations, and methods.
    private ColorSensor GripperColorSensor;
    private RevBlinkinLedDriver BlinkinLEDCtrl;
    private double colorGain = 1.05;

    public void initBlinkin() {
        GripperColorSensor = hardwareMap.get(ColorSensor.class, "GripperColorSensor");
        BlinkinLEDCtrl = hardwareMap.get(RevBlinkinLedDriver.class, "BlinkinLEDCtrl");
    }
    public void blinkinStatic() {
        BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }

    // Detect the color of the sample that is held by the gripper.
    private void colorDetect() {
        NormalizedRGBA myNormalizedColors;
        int myColor;
        float hue;
        float saturation;
        float value;

        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
        ((NormalizedColorSensor) GripperColorSensor).setGain((float) colorGain);
        // Save the color sensor data as a normalized color value. It's recommended
        // to use Normalized Colors over color sensor colors is because Normalized
        // Colors consistently gives values between 0 and 1, while the direct
        // Color Sensor colors are dependent on the specific sensor you're using.
        myNormalizedColors = ((NormalizedColorSensor) GripperColorSensor).getNormalizedColors();
        // Convert the normalized color values to an Android color value.
        myColor = myNormalizedColors.toColor();
        // Use the Android color value to calculate the Hue, Saturation and Value color variables.
        // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
        hue = JavaUtil.rgbToHue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        saturation = JavaUtil.rgbToSaturation(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        value = JavaUtil.rgbToValue(Color.red(myColor), Color.green(myColor), Color.blue(myColor));
        if (hue > 0 && hue <= 59 && saturation != 0) {
            telemetry.addData("color is ", "red");
            BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else if (hue >= 65 && hue <= 105 && saturation != 0) {
            telemetry.addData("color is ", "yellow");
            BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        } else if (hue >= 180 && hue <= 240 && saturation != 0) {
            telemetry.addData("color is ", "blue");
            BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else if (hue == 0 && saturation == 0 && value == 0) {
            telemetry.addData("color is ", "none");
            BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
        telemetry.update();
    }

//***************************** End Color and Blinkin ****************************************
//
//
//********************* Begin Pedro Pathing Pose and Chains **********************************
    // All associated Pedro variables, declarations, and methods.

    private Follower follower;
    private Path scorePreload;
    private PathChain threeSamplePush, scoreOntoBar;
    public int state = 0;
    //set poses
    private Pose startingPose = new Pose(8.5,66, Math.toRadians(0));
    private Pose firstOnBar = new Pose(24,66,Math.toRadians(0));
    private Pose secondOnBar = new Pose(24,68,Math.toRadians(0));
    private Pose thirdOnBar = new Pose(24,70,Math.toRadians(0));
    private Pose fourthOnBar = new Pose(24,72,Math.toRadians(0));
    private Pose fifthOnBar = new Pose(24,74,Math.toRadians(0));

    private Pose firstSampleLineup = new Pose(58,28,Math.toRadians(0));
    private Pose secondSampleLineup = new Pose(58,20,Math.toRadians(0));
    private Pose thirdSampleLineup = new Pose(58,13.5,Math.toRadians(0));
    private Pose firstSampleback = new Pose(26,28,Math.toRadians(0));
    private Pose secondSampleback = new Pose(26,20,Math.toRadians(0));
    private Pose thirdSampleback = new Pose(26,13.5,Math.toRadians(0));
    private Pose wallPickup = new Pose(24,28,Math.toRadians(180));

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startingPose),new Point(firstOnBar)));
        scorePreload.setConstantHeadingInterpolation(firstOnBar.getHeading());

        threeSamplePush = follower.pathBuilder()
                //curve from first place to in front of first sample
                .addPath(new BezierCurve(
                                new Point(firstOnBar),
                                new Point(28,20, Point.CARTESIAN),
                                new Point(62,46, Point.CARTESIAN),
                                new Point(firstSampleLineup))
                        //).setLinearHeadingInterpolation(firstOnBar.getHeading(),firstSampleLineup.getHeading())
                ).setConstantHeadingInterpolation(firstSampleLineup.getHeading())
                //push first sample
                .addPath(new BezierLine(
                        new Point(firstSampleLineup),
                        new Point(firstSampleback))
                ).setConstantHeadingInterpolation(firstSampleback.getHeading())
                //move in front of second sample
                .addPath(new BezierCurve(
                        new Point(firstSampleback),
                        new Point(51,26, Point.CARTESIAN),
                        new Point(60,26, Point.CARTESIAN),
                        new Point(secondSampleLineup))
                ).setConstantHeadingInterpolation(secondSampleLineup.getHeading())
                // push second sample
                .addPath(new BezierLine(
                        new Point(secondSampleLineup),
                        new Point(secondSampleback))
                ).setConstantHeadingInterpolation(secondSampleback.getHeading())
                // move in front of third sample
                .addPath(new BezierCurve(
                        new Point(secondSampleback),
                        new Point(51,16, Point.CARTESIAN),
                        new Point(60,16, Point.CARTESIAN),
                        new Point(thirdSampleLineup))
                ).setConstantHeadingInterpolation(thirdSampleLineup.getHeading())
                // push third sample
                .addPath(new BezierLine(
                        new Point(thirdSampleLineup),
                        new Point(thirdSampleback))
                ).setConstantHeadingInterpolation(thirdSampleback.getHeading())
                //move to first pickup
                .addPath(new BezierLine(
                        new Point(thirdSampleback),
                        new Point(wallPickup))
                ).setLinearHeadingInterpolation(thirdSampleback.getHeading(), wallPickup.getHeading())
                .setPathEndTimeoutConstraint(50)
                .build();
    }
    private void pathUpdate() {
        switch (state) {
            case 0:
                setLiftArmSlidePreSpecScorePos(); // slide and arm
                setRotationPosition(0.7);// gripper rotation
                follower.followPath(scorePreload);
                if (SlideMotor.getCurrentPosition() > 200) {
                    setOrientationPosition(0.5);
                    state = 1;
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    gripperIntake();
                    setSlideToPushSpecPos();
                    state = 2;
                }
                break;
            case 2:
                if (SlideMotor.getCurrentPosition() > (SlideMotor.getTargetPosition()-50) && !follower.isBusy()) {
                    gripperStop();
                    moveSlideArm(200);
                    setLiftArmSlideTravelPos();
                    setGripperHomePosition();
                    follower.followPath(threeSamplePush);
                    state = 7;
                }
                break;
        }
    }
    @Override
    public void init() {
        initLiftArmSlide(); // init arm
        initGripper(); // init gripper
        initClimb(); // init the climb motor
        initBlinkin(); // init blinkin
        setGripperHomePosition(); // set gripper position
        homeChainLiftArm(); // home the slide and chain motor
        lowerClimbingLift(); // home climbing lift
        Constants.setConstants(FConstants.class, LConstants.class); // set pedro constants

        follower = new Follower(hardwareMap); // create the follower
        follower.setStartingPose(startingPose); // set the robots starting pose
        buildPaths(); // build the paths

    }
    @Override
    public void loop() {
        follower.update();
        pathUpdate();
    }
}
//********************** End Pedro Pathing Pose and Chains ***********************************