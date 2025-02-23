package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "LeftFront";
        FollowerConstants.leftRearMotorName = "LeftBack";
        FollowerConstants.rightFrontMotorName = "RightFront";
        FollowerConstants.rightRearMotorName = "RightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;

        FollowerConstants.mass = 12.198;

        FollowerConstants.xMovement = 73.42930255869258;
        FollowerConstants.yMovement = 49.08626166756703;

        FollowerConstants.forwardZeroPowerAcceleration = -43.93091970409127;
        FollowerConstants.lateralZeroPowerAcceleration = -82.12548693246482;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.175,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.1,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID
        FollowerConstants.translationalPIDFFeedForward = 0;

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.headingPIDFFeedForward = 0;

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0175,0,0.0005,0.6,0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1,0,0,0.6,0); // Not being used, @see useSecondaryDrivePID
        FollowerConstants.drivePIDFFeedForward = 0;
        FollowerConstants.driveKalmanFilterParameters.dataCovariance = 1;
        FollowerConstants.driveKalmanFilterParameters.modelCovariance = 3;

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.0005;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.driveKalmanFilterParameters.dataCovariance = 0.5;
        FollowerConstants.driveKalmanFilterParameters.modelCovariance = 3;
    }
}
