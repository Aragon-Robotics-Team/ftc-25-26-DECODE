package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@TeleOp(name = "Limelight Detection Test", group = "Test")
public class LimelightDetectionTest extends LinearOpMode {

    private LimelightSubsystem limelight;

    // Simulation variables for Fusion testing
    // We pretend the robot thinks it is at (0,0) facing 0 degrees
    // to see what correction Limelight suggests based on where it actually is.
    private double simPinpointX = 0.0;
    private double simPinpointY = 0.0;
    private double simPinpointH = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Telemetry for Dashboard + Driver Station
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Subsystem
        limelight = new LimelightSubsystem(hardwareMap);

        telemetry.addLine("Limelight Initialized. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get latest result once per loop to ensure consistency
            LLResult result = limelight.getResult();

            telemetry.addData("Status", "Running");
            telemetry.addData("Pipeline Index", result != null ? result.getPipelineIndex() : "N/A");

            if (result != null && result.isValid()) {
                // ----------------------------------------------------
                // 1. AprilTag Detection Test
                // ----------------------------------------------------
                Pose robotPose = limelight.detectRobotPosition(result);
                Integer motifId = limelight.detectMotif(result);

                if (robotPose != null) {
                    telemetry.addData(">> VISION POSE (Inches)",
                            "X: %.2f, Y: %.2f", robotPose.getX(), robotPose.getY());
                } else {
                    telemetry.addData(">> VISION POSE", "No localization data");
                }

                if (motifId != null) {
                    telemetry.addData(">> MOTIF ID", motifId);
                }

                // ----------------------------------------------------
                // 2. Fusion Logic Test (Simulated)
                // ----------------------------------------------------
                // Press A to simulate a "Fusion Check" assuming robot thinks it is at 0,0
                if (gamepad1.a) {
                    Pose correction = limelight.getFusionCorrection(simPinpointX, simPinpointY, simPinpointH);

                    if (correction != null) {
                        telemetry.addData(">> FUSION SUGGESTION", "Reset Odo to: X=%.2f, Y=%.2f",
                                correction.getX(), correction.getY());
                    } else {
                        telemetry.addData(">> FUSION SUGGESTION", "No Correction (Drift too high or invalid)");
                    }
                }

                // ----------------------------------------------------
                // 3. Artifact/Sample Detection Test
                // ----------------------------------------------------
                Double artifactHeading = limelight.getArtifactHeading(result);
                Double artifactDist = limelight.getArtifactDistance(result);

                if (artifactHeading != null) {
                    telemetry.addData(">> ARTIFACT", "Heading: %.2f deg | Dist: %.2f in",
                            artifactHeading, artifactDist);
                } else {
                    telemetry.addData(">> ARTIFACT", "None detected");
                }

            } else {
                telemetry.addLine("No Valid Limelight Result (Check pipeline config)");
            }

            telemetry.addLine("\nControls:");
            telemetry.addLine("Hold A: Test Fusion Logic (Simulates robot thinking it is at 0,0)");

            telemetry.update();
        }
    }
}