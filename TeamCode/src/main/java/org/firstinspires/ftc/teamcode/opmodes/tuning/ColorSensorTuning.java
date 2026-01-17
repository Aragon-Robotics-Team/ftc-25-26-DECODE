package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;

import java.util.Locale;

@Config
@TeleOp(name = "Color Sensor Tuning", group = "Tuning")
public class ColorSensorTuning extends OpMode {

    private ColorSensorsSubsystem colorSubsystem;
    public GamepadEx driver1;
    float value = 2.0f;

    @Override
    public void init() {
        colorSubsystem = new ColorSensorsSubsystem(hardwareMap);
        driver1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to tune color sensors.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. READ DATA (Manually update each sensor)
        colorSubsystem.updateSensor1();
        colorSubsystem.updateSensor2();
        colorSubsystem.updateBack();

        // 2. GET HSV VALUES (Convert raw result to HSV for display)
        float[] hsv1 = ColorSensorsSubsystem.rgbToHsv(colorSubsystem.getIntakeSensor1Result());
        float[] hsv2 = ColorSensorsSubsystem.rgbToHsv(colorSubsystem.getIntakeSensor2Result());
        float[] hsvBack = ColorSensorsSubsystem.rgbToHsv(colorSubsystem.getBackResult());

        // get and set gain
        if (gamepad1.dpad_up) {
            value++;
            colorSubsystem.setGain(colorSubsystem.getIntakeSensor1(), value);
            colorSubsystem.setGain(colorSubsystem.getIntakeSensor2(), value);
        }
        if (gamepad1.dpad_down) {
            value--;
            if (value < 1.0f) {value = 1.0f;}
            colorSubsystem.setGain(colorSubsystem.getIntakeSensor1(), value);
            colorSubsystem.setGain(colorSubsystem.getIntakeSensor2(), value);

        }

        // 3. DISPLAY DATA
        telemetry.addData("Current Gain", value);

        // --- INTAKE SENSOR 1 ---
        telemetry.addLine("--- INTAKE SENSOR 1 ---");
        telemetry.addData("HSV", formatHSV(hsv1));
        telemetry.addData("Is Green?", ColorSensorsSubsystem.colorIsGreenIntake(colorSubsystem.getIntakeSensor1Result()));
        telemetry.addData("Is Purple?", ColorSensorsSubsystem.colorIsPurpleIntake(colorSubsystem.getIntakeSensor1Result()));
        telemetry.addLine();

        // --- INTAKE SENSOR 2 ---
        telemetry.addLine("--- INTAKE SENSOR 2 ---");
        telemetry.addData("HSV", formatHSV(hsv2));
        telemetry.addData("Is Green?", ColorSensorsSubsystem.colorIsGreenIntake(colorSubsystem.getIntakeSensor2Result()));
        telemetry.addData("Is Purple?", ColorSensorsSubsystem.colorIsPurpleIntake(colorSubsystem.getIntakeSensor2Result()));
        telemetry.addLine();

        // --- BACK SENSOR ---
        telemetry.addLine("--- BACK SENSOR ---");
        telemetry.addData("HSV", formatHSV(hsvBack));
        telemetry.addData("Is Green?", ColorSensorsSubsystem.colorIsGreenBack(colorSubsystem.getBackResult()));
        telemetry.addData("Is Purple?", ColorSensorsSubsystem.colorIsPurpleBack(colorSubsystem.getBackResult()));

        telemetry.update();
    }

    private String formatHSV(float[] hsv) {
        return String.format(Locale.US, "H: %.0f,  S: %.2f,  V: %.3f", hsv[0], hsv[1], hsv[2]);
    }
}