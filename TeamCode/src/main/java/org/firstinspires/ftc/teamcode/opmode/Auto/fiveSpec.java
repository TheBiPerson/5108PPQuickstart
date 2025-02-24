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

@Autonomous(name = "fiveSpec", group = "auto", preselectTeleOp = "Teleop")
public class fiveSpec extends OpMode {

    private ElapsedTime timer = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer;
    private LiftArmSlide liftArmSlide;
    private Gripper gripper;
    private int pathState;

    private PathChain preload, pushSample1Path, pushSample2Path, pushSample3Path, combinedPush, score1, return1, score2, return2, score3, return3, score4, return4, shift, inter;
    private Pose startingPose =  new Pose   (7, 65, Math.toRadians(0));

    private double wall_intake = 7.5;
    private double subX = 39;


    /**
     * Build our complex path sequence using the same calls
     * that were in the GeneratedPath constructor.
     */
    public void buildPaths() {
        // Preload path (Line 1)
        preload = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(7.338, 65.859, Math.toRadians(0))),
                        new Point(new Pose(40, 65.859, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        combinedPush = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(40.000, 65.859, Math.toRadians(0))),
                        new Point(new Pose(34.000, 67.000, Math.toRadians(0))),
                        new Point(new Pose(0.000, 48.000, Math.toRadians(0))),
                        new Point(new Pose(50, 30.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(50, 30.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 20.000, Math.toRadians(0))),
                        new Point(new Pose(22, 20.000, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 20.000, Math.toRadians(0))),
                        new Point(new Pose(65.000, 28, Math.toRadians(0))),
                        new Point(new Pose(50, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(new Pose(50, 13.5, Math.toRadians(0))),
                        new Point(new Pose(22, 13.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(new Pose(22, 13.5, Math.toRadians(0))),
                        new Point(new Pose(65.000, 16.250, Math.toRadians(0))),
                        new Point(new Pose(50, 7.5, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(50.000, 7.5, Point.CARTESIAN),
                                new Point(19, 7.5, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        inter = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(19, 7.5, Point.CARTESIAN),
                                new Point(17.3, 13, Point.CARTESIAN),
                                new Point(wall_intake, 28, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(new Pose(wall_intake, 28, Math.toRadians(0))),
                        new Point(new Pose(subX, 13, Math.toRadians(0))),
                        new Point(new Pose(wall_intake, 70, Math.toRadians(0))),
                        new Point(new Pose(subX, 70, Math.toRadians(0)))))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return1 = follower.pathBuilder()
                .addPath(
                    new BezierCurve(
                        new Point(subX, 70, Point.CARTESIAN),
                        new Point(wall_intake, 70, Point.CARTESIAN),
                        new Point(subX, 28, Point.CARTESIAN),
                        new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(wall_intake, 28.000, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake, 68, Point.CARTESIAN),
                                new Point(subX, 68.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(subX, 68, Point.CARTESIAN), new Point(subX, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        return2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(subX, 68.5, Point.CARTESIAN),
                                new Point(wall_intake, 66, Point.CARTESIAN),
                                new Point(subX, 28, Point.CARTESIAN),
                                new Point(wall_intake, 28.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    // Start following our newly built path
                    follower.followPath(preload, 1, true);
                    gripper.setGripperHomePosition();

                }
                if (pathTimer.getElapsedTimeSeconds() > 0.2 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                    liftArmSlide.moveSlideArm(500);
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    liftArmSlide.moveSlideArm(230);
                    if (liftArmSlide.slidePos < 250) {
                        gripper.gripperIntake();
                        setPathState(pathState + 1);
                    }
                }
                break;
            case 1: // Place specimen onto rung
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 5) {
                    liftArmSlide.moveSlideArm(0);
                    liftArmSlide.rotateLiftArm(90, 0.5);
                    gripper.setWallIntakePosition();
                    follower.followPath(combinedPush, 1,false);
                    setPathState();
            }
                break;
            case 2: // Travel to wall specimen
                if(!follower.isBusy()) {
                    follower.followPath(inter,0.6, true);
                    setPathState(4);
                }
                break;
            case 4: // Specimen wall intake
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 1.2) {
                    gripper.gripperOuttake();
                    liftArmSlide.moveSlideArm(50);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(score1, true); // Follow score1 path
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3 && pathTimer.getElapsedTimeSeconds() < 1) {
                    liftArmSlide.moveSlideArm(110);
                    gripper.setGripperHomePosition(); // Adjust gripper for scoring
                         // Set initial slide position
                }
                

                if (pathTimer.getElapsedTimeSeconds() > 1.2 && pathTimer.getElapsedTimeSeconds() < 2.5) {
                    liftArmSlide.moveSlideArm(460); // Lift slide to scoring position
                }

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    liftArmSlide.moveSlideArm(230); // Lower slide slightly for object release
                    if (liftArmSlide.slidePos < 250) {
                        gripper.gripperIntake(); // Release object
                        setPathState(pathState + 1); // Transition to return state
                    }
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    liftArmSlide.moveSlideArm(0); // Reset slide to initial position
                    liftArmSlide.rotateLiftArm(90, 0.5); // Reset lift arm to initial position
                    gripper.setWallIntakePosition(); // Prepare gripper for intake
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
                    liftArmSlide.moveSlideArm(230);
                    if (liftArmSlide.slidePos < 250) {
                        gripper.gripperIntake();
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

        liftArmSlide = new LiftArmSlide(hardwareMap, telemetry);
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
        gripper.setGripperHomePosition();
        pathTimer.resetTimer();
        setPathState(0);
    }

}