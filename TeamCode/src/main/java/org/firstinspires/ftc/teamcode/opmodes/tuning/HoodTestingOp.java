package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@TeleOp(name = "Hood Test", group = "Tuning")
public class HoodTestingOp extends OpMode {
    private HoodSubsystem hood;

    @Override
    public void init() {
        hood = new HoodSubsystem(hardwareMap);
        // Initialize Telemetry for Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    double currentAngle = 0.0;
    @Override
    public void loop() {
        if (gamepad1.a) {
            currentAngle += 0.5;
        } else if (gamepad1.b) {
            currentAngle -= 0.5;
        }
        hood.setHoodAngle(currentAngle);
        telemetry.addData("Current Hood Angle as in code (Math.toDegrees): ", Math.toDegrees(currentAngle));
        telemetry.addData("Current Hood Angle as in code (no Math.toDegrees): ", currentAngle);
    }
}
