package org.firstinspires.ftc.teamcode.subsystems;

//import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class ColorSensorsSubsystem extends SubsystemBase {
    private NormalizedColorSensor intakeSensor1;
    private NormalizedRGBA intakeSensor1Result = null;
    private NormalizedColorSensor intakeSensor2;
    private NormalizedRGBA intakeSensor2Result = null;

    private NormalizedColorSensor backSensor;
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


    public ColorSensorsSubsystem(final HardwareMap hMap) {
        intakeSensor1 = hMap.get(NormalizedColorSensor.class, "colori1");
        intakeSensor1.setGain(17.0f);

        intakeSensor2 = hMap.get(NormalizedColorSensor.class, "colori2");
        intakeSensor2.setGain(17.0f);

        backSensor = hMap.get(NormalizedColorSensor.class, "colorb");
        backSensor.setGain(17.0f);
    }
    public void setGain(NormalizedColorSensor sensor, float gain) {
        sensor.setGain(gain);
    }
    public NormalizedColorSensor getIntakeSensor1() {return intakeSensor1;}
    public NormalizedColorSensor getIntakeSensor2() {return intakeSensor2;}
    public NormalizedColorSensor getBackSensor() {return backSensor;}
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
        return colorIsBall(intakeSensor1Result)||colorIsBall(intakeSensor2Result);
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

