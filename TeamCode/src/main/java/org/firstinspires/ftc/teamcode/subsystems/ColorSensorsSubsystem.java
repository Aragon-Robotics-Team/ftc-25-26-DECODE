package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorsSubsystem extends SubsystemBase {
    private RevColorSensorV3 intakeSensor1;
    private NormalizedRGBA intakeSensor1Result = null;
    private RevColorSensorV3 intakeSensor2;
    private NormalizedRGBA intakeSensor2Result = null;

    private RevColorSensorV3 backSensor;
    private NormalizedRGBA backResult = null;


    public final static float[] intakeGreenHigherHSV = {173f, 0.75f, 0.272f};
    public final static float[] intakeGreenLowerHSV  = {133f, 0.45f, 0.0f};

    public final static float[] intakePurpleHigherHSV = {240f, 0.4f, 0.242f};
    public final static float[] intakePurpleLowerHSV  = {150f, 0.15f, 0.0f};

    public final static float[] backGreenHigherHSV = {178f, 0.80f, 0.83f};
    public final static float[] backGreenLowerHSV  = {138f, 0.35f, 0.3f};

    public final static float[] backPurpleHigherHSV = {220f, 0.60f, 0.73f};
    public final static float[] backPurpleLowerHSV  = {200f, 0.20f, 0.3f};

    public final static float[] whiteLowerHSV = {0f, 0.99f, 0.99f};
    public final static float[] whiteHigherHSV = {360f, 1f, 1f};

    public final static double alphaHigher1 = 0.50;
    public final static double alphaLower1 = 0.25;
    public final static double alphaHigher2 = 0.25;
    public final static double alphaLower2 = 0.0;

    public final static double proximityHigher1 = 1.4;
    public final static double proximityLower1 = 1.0;
    public final static double proximityHigher2 = 3.0;
    public final static double proximityLower2 = 2.3;

    public ColorSensorsSubsystem(final HardwareMap hMap) {
        intakeSensor1 = hMap.get(RevColorSensorV3.class, "colori1");
        intakeSensor1.setGain(17.0f);

        intakeSensor2 = hMap.get(RevColorSensorV3.class, "colori2");
        intakeSensor2.setGain(17.0f);

        backSensor = hMap.get(RevColorSensorV3.class, "colorb");
        backSensor.setGain(17.0f);
    }
    public void setGain(RevColorSensorV3 sensor, float gain) {
        sensor.setGain(gain);
    }
    public RevColorSensorV3 getIntakeSensor1() {return intakeSensor1;}
    public RevColorSensorV3 getIntakeSensor2() {return intakeSensor2;}
    public RevColorSensorV3 getBackSensor() {return backSensor;}
    public NormalizedRGBA getIntakeSensor1Result() {
        return intakeSensor1Result;
    }
    public NormalizedRGBA getIntakeSensor2Result() {
        return intakeSensor2Result;
    }
    public NormalizedRGBA getBackResult() {
        return backResult;
    }
    public void updateSensor1() {
        intakeSensor1Result = intakeSensor1.getNormalizedColors();
    }
    public void updateSensor2() {
        intakeSensor2Result = intakeSensor2.getNormalizedColors();
    }
    public void updateBack() {
        backResult = backSensor.getNormalizedColors();
    }
    public static boolean colorIsGreenIntake(NormalizedRGBA color) {
        return colorInRange(rgbToHsv(color), intakeGreenLowerHSV, intakeGreenHigherHSV);
    }
    public static boolean colorIsPurpleIntake(NormalizedRGBA color) {
        return colorInRange(rgbToHsv(color), intakePurpleLowerHSV, intakePurpleHigherHSV);
    }
    public static boolean colorIsGreenBack(NormalizedRGBA color) {
        return colorInRange(rgbToHsv(color), backGreenLowerHSV, backGreenHigherHSV);
    }
    public static boolean colorIsPurpleBack(NormalizedRGBA color) {
        return colorInRange(rgbToHsv(color), backPurpleLowerHSV, backPurpleHigherHSV);
    }
    public static boolean colorIsWhite(NormalizedRGBA color) {
        return colorInRange(rgbToHsv(color), whiteLowerHSV, whiteHigherHSV);
    }
    public static boolean colorIsBall(NormalizedRGBA color) {
        return colorIsGreenIntake(color) || colorIsPurpleIntake(color) || colorIsGreenBack(color) || colorIsPurpleBack(color) || colorIsWhite(color);
    }
    public boolean doesLastResultHaveBall() {
        return colorIsBall(intakeSensor1Result)||colorIsBall(intakeSensor2Result)||isClose1(intakeSensor1)||isClose2(intakeSensor2);
    }
    public static float getAlphaValue(NormalizedRGBA normalizedRGBA) {
        return normalizedRGBA.alpha;
    }
    public static boolean alphaIsNotClear1(NormalizedRGBA normalizedRGBA) {
        return (alphaLower1 < normalizedRGBA.alpha && normalizedRGBA.alpha < alphaHigher1);
    }
    public static boolean alphaIsNotClear2(NormalizedRGBA normalizedRGBA) {
        return (alphaLower2 < normalizedRGBA.alpha && normalizedRGBA.alpha < alphaHigher2);
    }
    public static double getProximity(RevColorSensorV3 colorSensor) {
        return (colorSensor.getDistance(DistanceUnit.INCH));
    }
    public static boolean isClose1(RevColorSensorV3 colorSensor) {
        return (proximityLower1 < colorSensor.getDistance(DistanceUnit.INCH) && colorSensor.getDistance(DistanceUnit.INCH) < proximityHigher1);
    }
    public static boolean isClose2(RevColorSensorV3 colorSensor) {
        return (proximityLower2 < colorSensor.getDistance(DistanceUnit.INCH) && colorSensor.getDistance(DistanceUnit.INCH) < proximityHigher2);
    }

    private static boolean colorInRange(float[] colorHSV, float[] min, float[] max) {
        return
                min[0] <= colorHSV[0] && colorHSV[0] <= max[0] && //Red is within min and max range
                        min[1] <= colorHSV[1] && colorHSV[1] <= max[1] && //Green is within min and max range
                        min[2] <= colorHSV[2] && colorHSV[2] <= max[2];   //brue is within the range,
    }
    // Function to convert RGB to HSV
    public static float[] rgbToHsv(NormalizedRGBA normalizedRGBA) {
        float r = normalizedRGBA.red;
        float g = normalizedRGBA.green;
        float b = normalizedRGBA.blue;
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;
        float h = 0, s = 0, v = max; // set Value
        if (delta != 0) {
            //calc Saturation
            s = delta / max;
            //calc Hue
            if (r == max) {
                h = (g - b) / delta;
            } else if (g == max) {
                h = 2 + (b - r) / delta;
            } else {
                h = 4 + (r - g) / delta;
            }
            h *= 60; // comvert to 360

            if (h < 0) { // make sure it's always positive
                h += 360;
            }
        }
        return new float[] {h, s, v};
    }
}

