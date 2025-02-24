package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "BESA_Five_Spec_Auto", group = "auto", preselectTeleOp = "Teleop")
public class BESA_Five_Spec_Auto extends OpMode {

    private int barX = 30;
    private int samplePushX = 16;
    private Follower follower;
    private Path scorePreload;
    private PathChain threeSamplePush;
    private PathChain scoreOntoBar;
    public int state = 0;
    //set poses
    private Pose startingPose = new Pose(8.5,66, Math.toRadians(0));
    private Pose firstOnBar = new Pose(26,66,Math.toRadians(0));
    private Pose secondOnBar = new Pose(26,68,Math.toRadians(0));
    private Pose thirdOnBar = new Pose(26,70,Math.toRadians(0));
    private Pose fourthOnBar = new Pose(26,72,Math.toRadians(0));
    private Pose fifthOnBar = new Pose(26,74,Math.toRadians(0));

    private Pose firstSampleLineup = new Pose(58,26,Math.toRadians(0));
    private Pose secondSampleLineup = new Pose(58,16,Math.toRadians(0));
    private Pose thirdSampleLineup = new Pose(58,8.5,Math.toRadians(0));
    private Pose firstSampleback = new Pose(16,26,Math.toRadians(0));
    private Pose secondSampleback = new Pose(16,16,Math.toRadians(0));
    private Pose thirdSampleback = new Pose(16,8.5,Math.toRadians(0));
    private Pose wallPickup = new Pose(24,24,Math.toRadians(180));


    private void pathUpdate() {
        switch (state) {
            case 0:
                follower.followPath(scorePreload);
                state = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(threeSamplePush);
                    state = 7;
                    break;
                }
        }
    }
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        scorePreload = new Path(new BezierLine(new Point(startingPose),new Point(firstOnBar)));
        scorePreload.setConstantHeadingInterpolation(firstOnBar.getHeading());

        threeSamplePush = follower.pathBuilder()
            //curve from first place to infront of first sample
            .addPath(new BezierCurve(
                 new Point(firstOnBar),
                 new Point(28,20, Point.CARTESIAN),
                 new Point(62,46, Point.CARTESIAN),
                 new Point(firstSampleLineup))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            //push first sample
            .addPath(new BezierLine(
                  new Point(firstSampleLineup),
                  new Point(firstSampleback))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            //move in front of second sample
            .addPath(new BezierCurve(
                  new Point(firstSampleback),
                  new Point(51,26, Point.CARTESIAN),
                  new Point(60,26, Point.CARTESIAN),
                  new Point(secondSampleLineup))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            .setPathEndTimeoutConstraint(50)
            .build();

        follower.followPath(scorePreload);

    }
    @Override
    public void loop() {
        follower.update();
        pathUpdate();
    }
}
