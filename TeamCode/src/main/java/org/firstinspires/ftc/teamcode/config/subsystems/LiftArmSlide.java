package org.firstinspires.ftc.teamcode.config.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;

public class LiftArmSlide {
    private DcMotorEx ChainLiftMotor;
    private DcMotorEx SlideMotor;
    private TouchSensor SlideHomeMagTouch;
    private TouchSensor ChainLiftHomeTouch;

    private static final double[] autoLiftArmCoefficients = {1.26,0.126,0, 12.6};
    private static final double[] autoSlideCoefficients = {1.26,0.126,0, 12.6};

    public PIDFController slidePIDF;
    public PIDFController liftArmPIDF;
    private static double slideF;
    private static double liftArmF;
    public double slideTarget = 0;
    public double liftArmTarget = 0;
    public boolean slidesReached;
    public boolean liftArmReached;
    // Between retracted and extended
    public boolean slidesRetracted;
    public double liftArmPos;
    public double liftPos;
    public boolean pidfActive = true;

    public LiftArmSlide(HardwareMap hardwareMap, Telemetry telemetry) {

        slidePIDF = new PIDFController(autoSlideCoefficients[0], autoSlideCoefficients[1], autoSlideCoefficients[2], autoSlideCoefficients[3]);
        liftArmPIDF = new PIDFController(autoLiftArmCoefficients[0], autoLiftArmCoefficients[1], autoLiftArmCoefficients[2], autoLiftArmCoefficients[3]);
        slideF = autoSlideCoefficients[3];
        liftArmF = autoLiftArmCoefficients[3];

        ChainLiftHomeTouch = hardwareMap.get(TouchSensor.class, "ChainLiftHomeTouch");
        SlideHomeMagTouch = hardwareMap.get(TouchSensor.class, "SlideHomeMagTouch");

        ChainLiftMotor = hardwareMap.get(DcMotorEx.class, "ChainLiftMotor");
        SlideMotor = hardwareMap.get(DcMotorEx.class, "SlideMotor");
        ChainLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ChainLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        SlideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



        slidePIDF.setTolerance(15);
        liftArmPIDF.setTolerance(1);
        setSlideTarget(Math.round((float) SlideMotor.getCurrentPosition() / 42) * -1);
        setPivotTarget(liftArmPos());
    }

    public void setSlideTarget(double target) {
        this.slideTarget = Range.clip(target, 0, 1250);
        slidePIDF.setSetPoint(slideTarget);
    }

    public void setPivotTarget(double target) {
        this.liftArmTarget = Range.clip(target, 0, 121);
        liftArmPIDF.setSetPoint(liftArmTarget);
    }

    public void update() {
        liftPos = liftPos();
        liftArmPos = liftArmPos();
        /// slidePIDF.setF(slideF * Math.sin(Math.toRadians(liftArmPos)));
        double liftPower = slidePIDF.calculate(liftPos, slideTarget);
        slidesReached = slidePIDF.atSetPoint() || (liftPos >= slideTarget && slideTarget == 1050);
        slidesRetracted = slideTarget <= 0 && slideLimit.isPressed();

        liftArmPIDF.setF(liftArmF * Math.cos(Math.toRadians(liftArmPos)) * ((double) liftPos / 1250));
        double pivotPower = liftArmPIDF.calculate(liftArmPos, liftArmTarget);
        liftArmReached = liftArmPIDF.atSetPoint();

        // Just make sure it gets to fully retracted if target is 0
        if (slideTarget == 0 && !slidesReached) {
            liftPower -= 0.1;
        } else if (slideTarget >= 1050 && !slidesReached) {
            liftPower += 0.6;
        }

        if (pidfActive) {
            if (slidesRetracted) {
                SlideMotor.setPower(0);
            } else if (liftArmPos <= 10 && slidesReached) {
                SlideMotor.setPower(0);
            } else {
                SlideMotor.setPower(liftPower);
            }
        } else {
            if (slideLimit.isPressed()) {
                SlideMotor.setPower(0);
                SlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                slidePIDF.reset();
                pidfActive = true;
            } else {
                SlideMotor.setPower(-1);
            }
        }


        if ((slideTarget < 500 && liftArmTarget == 121 && liftArmReached) || (liftArmTarget <= 12 && liftArmReached)) {
            pivotPower = 0;
        }

        pivot.setPower(pivotPower);
    }

    public int liftPos() {
        return Math.round((float) SlideMotor.getCurrentPosition() / 42) * -1;
    }

    public int liftArmPos() {
        // int pos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360)) % 360 - 168;
        int pos = (int) (Math.round(pivotEncoder.getVoltage() / 3.2 * 360)) % 360 - 171;

        if (pos >= 360) {
            pos -= 360;
        } else if (pos < 0) {
            pos += 360;
        }
        if (pos > 345) {
            pos = 0;
        }
        return pos;
    }

}
