package org.firstinspires.ftc.teamcode.config.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class ColorBlink {
    private ColorSensor GripperColorSensor;
    private RevBlinkinLedDriver BlinkinLEDCtrl;
    private double colorGain = 1.05;

    public ColorBlink (HardwareMap hardwareMap) {
        // Map color devices
        BlinkinLEDCtrl = hardwareMap.get(RevBlinkinLedDriver.class, "BlinkinLEDCtrl");
        GripperColorSensor = hardwareMap.get(ColorSensor.class, "GripperColorSensor");
    }

    public void blinkinStatic() {
        BlinkinLEDCtrl.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
    }

    public void sampleColorID () {
        // ID the color of the sample
        colorDetect();
    }

    /**
     * Detect the color of the sample that is held by the gripper.
     */
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

}
