package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.teamcode.config.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.config.subsystems.LiftArmSlide;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

@Autonomous(name = "SCB_FiveSpec_Auto", group = "auto")
public class SCB_FiveSpec_Auto extends OpMode {

    private ElapsedTime Timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer;
    private LiftArmSlide liftArmSlide;
    private Gripper gripper;
    private int pathState;

    private PathChain preload, combinedPush, intake, score1, return1, score2, return2;
//    private PathChain preload, combinedPush, intake, score1, return1, score2, return2, score3, return3, score4, return4, shift;
    private Pose startingPose =  new Pose   (8.5, 66, Math.toRadians(0));
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


    private double wall_intakeX = 18;
    private double subX = 39;


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(8.5, 66, Math.toRadians(0))),
                        new Point(new Pose(26.0, 66.0, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // Path to push three samples to obs zone
        combinedPush = follower.pathBuilder()
                //curve from first place to in front of first sample
                .addPath(new BezierCurve(
                        new Point(new Pose(26.0, 66.0, Math.toRadians(90))),
                        new Point(new Pose(35.000, 53.000, Math.toRadians(90))),
                        new Point(new Pose(15.000, 37.000, Math.toRadians(90))),
                        new Point(new Pose(58, 30.000, Math.toRadians(90)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //push first sample
                .addPath(new BezierCurve(
                        new Point(new Pose(58, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(22, 20.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                //move in front of second sample
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 20.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(58, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // push second sample
                .addPath(new BezierLine(
                        new Point(new Pose(58, 13.5, Math.toRadians(0))),
                        new Point(new Pose(22, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // move in front of third sample
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 13.5, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(58, 8.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                // push third sample
                .addPath(new BezierLine(
                         new Point(58.000, 8.5, Point.CARTESIAN),
                         new Point(19, 7.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
                // The robot is now positioned in the obs zone after pushing three samples

        // This segment moves the robot to the wall pickup location and rotates to 180 deg.
        intake = follower.pathBuilder()
                .addPath(new BezierCurve(
                         new Point(19, 7.5, Point.CARTESIAN),
                         new Point(17.3, 13, Point.CARTESIAN),
                         new Point(wall_intakeX, 28, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(wall_intakeX, 28, Math.toRadians(0))),
                        new Point(new Pose(subX, 13, Math.toRadians(0))),
                        new Point(new Pose(wall_intakeX, 70, Math.toRadians(0))),
                        new Point(new Pose(subX, 70, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Point(subX, 70, Point.CARTESIAN),
                        new Point(wall_intakeX, 70, Point.CARTESIAN),
                        new Point(subX, 28, Point.CARTESIAN),
                        new Point(wall_intakeX, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(wall_intakeX, 28.000, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intakeX, 68, Point.CARTESIAN),
                                new Point(subX, 68.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, 68.0, Point.CARTESIAN),
                                new Point(wall_intakeX, 66, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intakeX, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, 1, true);
                    //gripper.setGripperHomePosition();

                }
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                    //liftArmSlide.moveSlideArm(500);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    //liftArmSlide.moveSlideArm(230);
                    if (liftArmSlide.slidePos < 250) {
                        //gripper.gripperIntake();
                        setPathState(pathState + 1);
                    }
                }
                break;
            case 1: // Place specimen onto rung
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                    //liftArmSlide.setLiftArmSlidePreSpecScorePos();
                    //gripper.setPreSpecimenPlacement();
                    follower.followPath(combinedPush, 1,false);
                    setPathState();
            }
                break;
            case 2: // Travel to wall specimen
                if(!follower.isBusy()) {
                    //liftArmSlide.setLiftArmSlideTravelPos();
                    follower.followPath(intake,0.6, true);
                    setPathState(4);
                }
                break;
            case 4: // Specimen wall intake
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.2) {
                    //gripper.gripperOuttake();
                    //liftArmSlide.moveSlideArm(50);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1, true); // Follow score1 path
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
                    //liftArmSlide.moveSlideArm(110);
                    //gripper.setGripperHomePosition(); // Adjust gripper for scoring
                         // Set initial slide position
                }
                

                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                    //liftArmSlide.moveSlideArm(460); // Lift slide to scoring position
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    //liftArmSlide.moveSlideArm(230); // Lower slide slightly for object release
                    if (liftArmSlide.slidePos < 250) {
                        //gripper.gripperIntake(); // Release object
                        setPathState(pathState + 1); // Transition to return state
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    //liftArmSlide.moveSlideArm(0); // Reset slide to initial position
                    //liftArmSlide.rotateLiftArm(90, 0.5); // Reset lift arm to initial position
                    //gripper.setWallIntakePosition(); // Prepare gripper for intake*/
                    follower.followPath(return1, true); // Follow return1 path
                    setPathState(pathState + 1); // Transition to the next scoring path
                }
                break;
            case 7:
                cycle();
                break;
            case 8:
                intake();
                break;
            case 9:
                cycle();
                break;
            case 10:
                intake();
                break;
            case 11:
                cycle();
                if (pathTimer.getElapsedTimeSeconds() > 4.4) {
                    //liftArmSlide.moveSlideArm(230);
                    if (liftArmSlide.slidePos < 250) {
                        //gripper.gripperIntake();
                        setPathState(pathState + 1); // Transition to next return
                    }
                }
                break;
            case 12:
                setPathState(-1);
                break;
            default:
                if (!follower.isBusy()) {
                    requestOpModeStop(); // Stop the OpMode if all states are complete
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setPathState() {
        pathState += 1;
        pathTimer.resetTimer();
    }

    private void cycle() {

        if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.8) {
            follower.followPath(score2, true); // Follow score2 path
        }
        if (pathTimer.getElapsedTimeSeconds() > 1.8 && pathTimer.getElapsedTimeSeconds() < 2.3) {
            gripper.gripperOuttake();
            liftArmSlide.moveSlideArm(110);
        }

        if (pathTimer.getElapsedTimeSeconds() > 2.3 && pathTimer.getElapsedTimeSeconds() < 3) {
            gripper.setPreSpecimenPlacement();
            liftArmSlide.moveSlideArm(100);
        }

        if (pathTimer.getElapsedTimeSeconds() > 3.2 && pathTimer.getElapsedTimeSeconds() < 4.3) {
            liftArmSlide.moveSlideArm(460);
        }

        if (pathTimer.getElapsedTimeSeconds() > 4.6) {
            liftArmSlide.moveSlideArm(230);
            if (liftArmSlide.slidePos < 250) {
                gripper.gripperIntake();
                setPathState(pathState + 1); // Transition to next return
            }
        }
    }

    private void intake() {
        if (!follower.isBusy() || follower.getVelocityMagnitude() < 0.05) {
            liftArmSlide.moveSlideArm(0);
            liftArmSlide.rotateLiftArm(90, 0.5);
            gripper.gripperOuttake();
            gripper.setWallIntakePosition();
            follower.followPath(return2, false); // Follow return2 path
            setPathState(pathState + 1); // Transition to next scoring path
        }
    }

    private void preCycle() {
        if (!follower.isBusy()) {
            gripper.gripperOuttake();
            liftArmSlide.moveSlideArm(50);
            setPathState(pathState + 1);
        }
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X Pos", follower.getPose().getX());
        telemetry.addData("Y Pos", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Path Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path Busy", follower.isBusy());


        telemetry.update();

        liftArmSlide.homeChainLiftArm();

        autonomousPathUpdate();
    }

    @Override
    public void init() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();

        // Initialize follower, slides, etc. as usual
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        // If needed, set a starting pose (only if your system requires it)
        // follower.setStartingPose(new Pose(0,0, 0));
        follower.setStartingPose(startingPose);
        follower.setMaxPower(1);

        liftArmSlide = new LiftArmSlide(hardwareMap);
        gripper = new Gripper(hardwareMap);

        gripper.setGripperHomePosition();
        liftArmSlide.homeChainLiftArm();

        // Build our newly incorporated multi-step path:
        buildPaths();

        //if (!liftArmSlide.slideLimit.isPressed()) {
          //  throw new IllegalArgumentException("Zero slides before init");
        //}
    }

    @Override
    public void start() {
        //gripper.setGripperHomePosition();
        pathTimer.resetTimer();
        setPathState(0);
    }

}