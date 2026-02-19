package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;
import static org.firstinspires.ftc.teamcode.RobotConstants.SHOOTER_ANGLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.AutoPoseSaver;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
import org.firstinspires.ftc.teamcode.commands.ShootSortedBallsCommandSequence;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.Arrays;

@Configurable
@Autonomous(name = "\uD83D\uDD34 Red 15 Close", group = "angryBirds", preselectTeleOp = "RedTeleOp")
public class RedVolumeCloseAuto extends CommandOpMode {
    //Rememeber, when changing to blue:
    //Reverse poses + headings
    //Pay attention to teleop saved pos offset at the bottom
    //Reverse sotm goal pos
    public static class Paths {
        //close autos
        public PathChain shootClosePreload;
        public PathChain intakeSecondRowClose;
        public PathChain shootSecondRowClose;
        public PathChain hitGateSecond;
        public PathChain intakeFirstRowClose;
        public PathChain shootFirstRowClose;
        public PathChain hitGateFirst;
        public PathChain intakeThirdRowClose;
        public PathChain shootThirdRowClose;
        public PathChain intakeRamp;
        public PathChain shootRamp;

        public static class Poses {
            public static final Pose LAUNCH = new Pose(86.8, 88.2, 0.799732);
            public static final Pose START = new Pose(121.48,123.623,0.79785);
            public static final Pose GATE = new Pose(132, 68);
            public static final Pose PARK_LAUNCH = new Pose(87.79745,110.10889, 0.497311);
        }

        public Paths(Follower follower) {
            shootClosePreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.START, Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Poses.START.getHeading(), Poses.LAUNCH.getHeading())
                    .build();
            intakeSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(87.6, 43),
                                    new Pose(126.13, 55)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            hitGateSecond = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.13, 55), Poses.GATE)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            shootSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.GATE,
                                    new Pose(91.5, 56),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Poses.LAUNCH.getHeading())
                    .build();

            intakeFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(100,79.5),
                                    new Pose(116, 84)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            hitGateFirst = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116, 84), Poses.GATE)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            shootFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126, 84), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.LAUNCH.getHeading())
                    .build();

            intakeThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(83, 11),
                                    new Pose(126, 36)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            shootThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120, 36), Poses.PARK_LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.PARK_LAUNCH.getHeading())
                    .build();

            intakeRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(108, 62),
                                    new Pose(133.7, 60.7)
                            )
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Math.toRadians(35))
                    .build();

            shootRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133, 60),
                                    new Pose(108, 62),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Poses.LAUNCH.getHeading())
                    .build();
        }
    }

    //Since intakeartifacts is called at very different times (called when on the gate, called before driving to the row of balls)
    //we might need to split it up.
    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                new WaitForColorCommand(colorsensor).withTimeout(3000),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(100),
                new WaitForColorCommand(colorsensor).withTimeout(500),
                new WaitCommand(100),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitForColorCommand(colorsensor).withTimeout(500)
        );
    }

    //Selection
    private enum AUTOS { //TODO: what is this ????
        //If we do do a switcher, maybe make it: a. 1x gate + 3rd row, b. 2x gate
        GATE_ONCE, INTAKE_GATE
    }
    final AUTOS CURRENTAUTO = AUTOS.INTAKE_GATE;

    public Pose currentPose;
    public RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{PURPLE, PURPLE,PURPLE};

    boolean sotm = false;

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private Follower follower;

    //update starting pose
    public static Pose startingPose = Paths.Poses.START;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    private LimelightSubsystem limelight;

    //debugging
    int debug = 0;
    InstantCommand setCount(int n) {
        return new InstantCommand(() -> debug = n);
    }
    void scanMotif() {
        limelight.takeSnapshot("MOTIF");
        Object motifid = limelight.detectMotif(limelight.getResult());
        if (motifid != null) {
            switch ((int) motifid) {
                case 21:
                    motif = new RobotConstants.BallColors[]{GREEN, PURPLE, PURPLE}; //changed to actual motif
                    break;
                case 22:
                    motif = new RobotConstants.BallColors[]{PURPLE, GREEN, PURPLE};
                    break;
                case 23:
                    motif = new RobotConstants.BallColors[]{PURPLE, PURPLE, GREEN}; //changed to actual motif
                    break;
            }
        }
    }
    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startingPose);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        intake.set(IntakeSubsystem.IntakeState.INTAKESTILL_ROLLERSSTILL);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        gate.down();
        led = new LEDSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        limelight = new LimelightSubsystem(hardwareMap);
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.APRILTAG);
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        colorsensor.updateBack();
        lastVoltageCheck.reset();
        Paths paths = new Paths(follower);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(115);
        SequentialCommandGroup auto = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    sotm = true;
                    gate.down();
                }),
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootClosePreload, true, 0.5)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.55).andThen(new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false))
                ),
                //Second row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeSecondRowClose)
                                .alongWith(new InstantCommand(() -> {
                                    intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN);
                                    sotm=false;
                                    shooter.setTargetTicks(1140);
                                }
                                ))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, GREEN, PURPLE});}),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.hitGateSecond).withTimeout(1500),
                new FollowPathCommand(follower, paths.shootSecondRowClose, true),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false),

                //First row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeFirstRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});}),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootFirstRowClose, true),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false),

                //ramp
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                        new FollowPathCommand(follower, paths.intakeRamp)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.85)
                                        .andThen(new InstantCommand(() -> follower.setMaxPower(0.4)))),
                        new WaitCommand(3000)
                                .andThen(intakeArtifacts()).withTimeout(5000)
                ),
                new FollowPathCommand(follower, paths.shootRamp, true),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false),

                //intake third row
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        new WaitCommand(2000)
                                .andThen(intakeArtifacts())
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, PURPLE, GREEN});}),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootThirdRowClose, true),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false)
        );

        schedule(new RunCommand(() -> follower.update()));
        schedule(new SequentialCommandGroup(auto));

    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        if (sotm) {
            shooter.setTargetLinearSpeed(calculateTargetVector2(follower, follower.getPose(), new Pose(144, 144), shooter).getMagnitude());
        }

        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        if ((Math.abs(spindexer.getCurrentPosition() - spindexer.getPIDSetpoint()) < 60)) {
            spindexer.handleUpdateArray(colorsensor.getIntakeSensor1Result(), colorsensor.getIntakeSensor2Result(), colorsensor.getBackResult());
        }
        if (shooter.getVelocityTicks() - shooter.getTargetTicks() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getVelocityTicks() - shooter.getTargetTicks() > 50) {
            led.setColor(LEDSubsystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubsystem.LEDState.GREEN);
        }


        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("Debug Counter", debug);
        telemetry.addData("Detected Motif", Arrays.toString(motif));

//        telemetry.addData("spindexer output", spindexer.getOutput());
//        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
//        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("spindexer's balls", Arrays.toString(spindexer.getBalls()));

        telemetry.addData("------------------",0);

        telemetry.addData("shooter target velocity", shooter.getTargetTicks());
        telemetry.addData("shooter actual velocity", shooter.getVelocityTicks());

        telemetry.addData("------------------",0);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",0);
        currentPose = follower.getPose();
        AutoPoseSaver.lastPose = currentPose;
        timer.reset();
        telemetry.update();
        super.run();

    }

    @Override
    public void end() {
        AutoPoseSaver.lastPose = currentPose;
        super.end();
    }
    public Vector calculateTargetVector2(Follower follower, Pose robotPose, Pose targetPose, ShooterSubsystem shooter) {
        // --- 0. CONFIGURATION ---
        // You must estimate your shooter's launch angle relative to the floor.
        // If your hood moves, calculate this based on hood position.
        // For fixed hoods, 45-60 degrees is common.
        double launchAngle = SHOOTER_ANGLE;
        double latency = 0.5; //Determine empirically

        // --- 1. GATHER CURRENT STATE ---
        Pose currentPose = robotPose;
        Vector v_robot = follower.getVelocity();
        double angularVel = follower.getAngularVelocity();
        Vector a_robot = follower.getAcceleration();

        //Position deadzone
        if (v_robot.getMagnitude() < 2.0) { // If moving less than 2 in/s
            v_robot = new Vector(0,0);
        }
        //Angle deadzone
        if (Math.abs(angularVel) < Math.toRadians(5)) { // If rotating less than 5 deg/s
            angularVel = 0;
        }

        // Offsets (Distance in inches from center of robot to shooter)
        double shooterOffsetX = 5.0;
        double shooterOffsetY = 0.0;

        // --- 2. PREDICT ROBOT POSE (Standard Kinematics) ---
        double futureHeading = currentPose.getHeading() + (angularVel * latency);

        // Position Prediction
        double predX = currentPose.getX() + (v_robot.getXComponent() * latency);
        double predY = currentPose.getY() + (v_robot.getYComponent() * latency);

        // --- 3. CALCULATE ROBOT VELOCITY AT MUZZLE (Standard Rigid Body) ---
        double futureVx = v_robot.getXComponent();
        double futureVy = v_robot.getYComponent();

        double cosH = Math.cos(futureHeading);
        double sinH = Math.sin(futureHeading);

        // Rotated offset
        double fieldOffsetX = (shooterOffsetX * cosH) - (shooterOffsetY * sinH);
        double fieldOffsetY = (shooterOffsetX * sinH) + (shooterOffsetY * cosH);

        // Tangential velocity
        double v_tangential_x = -angularVel * fieldOffsetY;
        double v_tangential_y =  angularVel * fieldOffsetX;

        // Total Robot Velocity Components (Cartesian)
        double finalRobotVx = futureVx + v_tangential_x;
        double finalRobotVy = futureVy + v_tangential_y;

        // Convert to Polar for Pedro Vector
        double robotVelMag = Math.hypot(finalRobotVx, finalRobotVy);
        double robotVelAngle = Math.atan2(finalRobotVy, finalRobotVx);
        Vector v_robot_total = new Vector(robotVelMag, robotVelAngle);

        // --- 4. SOLVE FOR SHOOTING VECTOR ---
        double shooterMsgX = predX + fieldOffsetX;
        double shooterMsgY = predY + fieldOffsetY;

        double dx = targetPose.getX() - shooterMsgX;
        double dy = targetPose.getY() - shooterMsgY;
        double dist = Math.hypot(dx, dy);
        double idealHeading = Math.atan2(dy, dx);

        // === THE FIX STARTS HERE ===

        // A. Get the Total Exit Speed required for this distance (from your lookup table/regression)
        double totalSpeedRequired = shooter.findSpeedFromDistance(dist);

        // B. "Flatten" this speed to the 2D floor plane
        //    We only want the horizontal component for vector math
        double horizontalSpeed = totalSpeedRequired * Math.cos(launchAngle);

        // C. Create the target vector using Horizontal Speed
        Vector v_target_horizontal = new Vector(horizontalSpeed, idealHeading);

        // D. Perform Vector Subtraction in the 2D plane
        //    (Horizontal Target) - (Horizontal Robot Motion) = (Horizontal Ball Launch Vector)
        Vector v_ball_horizontal = v_target_horizontal.minus(v_robot_total);

        // E. Convert the result back to Total Exit Speed for the flywheel
        //    Total = Horizontal / cos(theta)
        double finalHorizontalSpeed = v_ball_horizontal.getMagnitude();
        double finalTotalSpeed = finalHorizontalSpeed / Math.cos(launchAngle);
        if (dist > 110) {
            finalTotalSpeed = 620;
        }

        // Return a Vector with the NEW Total Speed and the CORRECTED heading
        return new Vector(finalTotalSpeed, v_ball_horizontal.getTheta());
    }
}
