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
    private Pose startingPose = new Pose(8.5,66, Math.toRadians(0));
    private Pose firstOnBar = new Pose(30,66,Math.toRadians(0));
    private int barX = 30;
    private int sampleStartX = 58;
    private int samplePushX = 16;
    private Follower follower;
    private Path scorePreload;
    private PathChain threeSamplePush;
    private PathChain scoreOntoBar;
    public int state = 0;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        scorePreload = new Path(new BezierLine(new Point(startingPose),new Point(firstOnBar)));


        threeSamplePush = follower.pathBuilder()
            //curve from first place to infront of first sample
            .addPath(new BezierCurve(
                 new Point(firstOnBar),
                 new Point(62,46, Point.CARTESIAN),
                 new Point(sampleStartX,26))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            //push first sample
            .addPath(new BezierLine(
                  new Point(58,26),
                  new Point(16,26))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            //move in front of second sample
            .addPath(new BezierCurve(
                  new Point(16,26),
                  new Point(51,26, Point.CARTESIAN),
                  new Point(60,26, Point.CARTESIAN),
                  new Point(sampleStartX,16))
            ).setConstantHeadingInterpolation(firstOnBar.getHeading())
            .setPathEndTimeoutConstraint(50)
            .build();

        follower.followPath(scorePreload);

    }
    @Override
    public void loop() {
        follower.update();
    }
}
