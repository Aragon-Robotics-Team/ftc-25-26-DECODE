package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

@Config
@TeleOp(name = "⚡ Spindexer PID Tuning", group = "Tuning")
public class SpindexerTuningOp extends LinearOpMode {

    // PIDF Coefficients (Editable via Dashboard)
    public static double p = 0.0159;
    public static double i = 0;
    public static double d = 0.0000114;
    public static double f = 0;

    // Test Targets
    public static double targetPosition = 0; // Use this to set a specific angle manually via Dashboard

    // Logic flags
    boolean lastA = false;
    boolean lastB = false;
    boolean lastX = false;
    boolean lastY = false;

    private SpindexerSubsystem spindexer;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Telemetry for Dashboard Graphing
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Hardware
        spindexer = new SpindexerSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Force initial PID set
        spindexer.setPIDCoefficients(p, i, d, f);

        telemetry.addLine("Ready to Tune Spindexer.");
        telemetry.addLine("A: +120 deg | B: -120 deg");
        telemetry.addLine("X: Go to Dashboard Target | Y: Reset to 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Voltage Compensation (Critical for consistent tuning)
            double currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            // Note: updatePIDVoltage in your subsystem modifies the internal kP based on voltage.
            // For raw tuning, we constantly re-apply the Dashboard values below,
            // so we rely on the subsystem to handle the math or we apply raw values.
            // Since your subsystem modifies kP internally, we will push the Dashboard values
            // into the subsystem's method which accepts raw P, I, D.

            spindexer.setPIDCoefficients(p, i, d, f);

            // 2. Gamepad Controls for Step Response

            // Button A: Increment by 120 (Standard Index)
            if (gamepad1.a && !lastA) {
                // Using -120 because your subsystem subtracts delta in moveSpindexerBy
                // Adjust sign here based on which way you want it to physically rotate
                spindexer.moveSpindexerBy(-120);
            }
            lastA = gamepad1.a;

            // Button B: Decrement by 120
            if (gamepad1.b && !lastB) {
                spindexer.moveSpindexerBy(120);
            }
            lastB = gamepad1.b;

            // Button X: Go to specific Dashboard Target
            if (gamepad1.x && !lastX) {
                spindexer.set(targetPosition);
            }
            lastX = gamepad1.x;

            // Button Y: Go to 0 (Home)
            if (gamepad1.y && !lastY) {
                spindexer.set(0);
            }
            lastY = gamepad1.y;

            // 3. Update Subsystem
            // IMPORTANT: In a LinearOpMode, we must manually call periodic()
            // because the Command Scheduler isn't running.
            spindexer.periodic();

            // 4. Telemetry Data
            // We output "Target" and "Current" first so they auto-graph in Dashboard
            telemetry.addData("Target Position", spindexer.getPIDSetpoint());
            telemetry.addData("Current Position", spindexer.getCurrentPosition());
            telemetry.addData("Error", spindexer.getPIDSetpoint() - spindexer.getCurrentPosition());

            telemetry.addData("Motor Output", spindexer.getOutput());
            telemetry.addData("Current (Amps)", spindexer.getSpindexerCurrentAmps());
            telemetry.addData("Voltage", currentVoltage);

            telemetry.addData("Is Near Target", spindexer.isNearTargetPosition());
            telemetry.update();
        }
    }
}