package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.AutoPoseSaver;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.commands.DeferredCommand;
import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
import org.firstinspires.ftc.teamcode.commands.ScanAndDriveToBallCommand;
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
import java.util.List;
import java.util.function.Function;

@Configurable
@Autonomous(name = "\uD83D\uDD34 Red Far Vision 15", group = "angryBirds", preselectTeleOp = "RedTeleOp")
public class RedFarVisionAuto extends CommandOpMode {
    public static class Paths {
        public PathChain shootFarPreload;
        public PathChain intakeThirdRowFar;
        public PathChain shootThirdRowFar;
        public PathChain intakeHp1;
        public PathChain intakeHpBack;
        public PathChain intakeHp2;
        public PathChain shootHpFar;
        public PathChain intakeOverflow;
        public PathChain shootOverflow;
        public PathChain parkFar;

        public PathChain toScanLow;
        public PathChain toScanMed;
        public PathChain toScanHigh;

        public static class Poses {
            public static final Pose LAUNCH = new Pose(89.4,18.5, Math.toRadians(63));
            public static final Pose START = new Pose(102.5, 8, Math.toRadians(90));
            public static final Pose SCAN_LOW = new Pose(91, 18.5, Math.toRadians(1));
            public static final Pose SCAN_MED = new Pose(91, 18.5, Math.toRadians(5));
            public static final Pose SCAN_HIGH = new Pose(91, 18.5, Math.toRadians(10));
            public static final Pose INTAKE_HP = new Pose(128, 10, Math.toRadians(0));
            public static final Pose INTAKE_HP2 = new Pose(128, 10, Math.toRadians(90));
        }

        public Paths(Follower follower) {
            shootFarPreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.START, Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Poses.START.getHeading(), Poses.LAUNCH.getHeading())
                    .build();

            intakeThirdRowFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(85, 32.6),
                                    new Pose(126.13, 35.82)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootThirdRowFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.13, 33.82), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.LAUNCH.getHeading())
                    .build();

            intakeHp1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, new Pose(132, 8))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            intakeHpBack = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(132, 8), new Pose(121,8))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeHp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(121,8), new Pose(132, 8))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootHpFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132, 8),
                                    new Pose(118.5,30),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Poses.LAUNCH.getHeading())
                    .build();

            intakeOverflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(144, 0),
                                    new Pose(144, 0),
                                    new Pose(130, 20),
                                    new Pose(136.3,33)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootOverflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(136.3,33), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Poses.LAUNCH.getHeading())
                    .build();

            toScanLow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, Poses.SCAN_LOW)
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Poses.SCAN_LOW.getHeading())
                    .build();
            toScanMed = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, Poses.SCAN_MED)
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Poses.SCAN_MED.getHeading())
                    .build();
            toScanHigh = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, Poses.SCAN_HIGH)
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Poses.SCAN_HIGH.getHeading())
                    .build();

            parkFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, new Pose(110, 8))
                    )
                    .setConstantHeadingInterpolation(Poses.LAUNCH.getHeading())
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
    private SequentialCommandGroup shootFourTimesWithDelay() {
        return new SequentialCommandGroup(
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(300),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(300),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(300),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(100)
        );
    }
    public Pose currentPose;
    public RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{PURPLE,PURPLE,PURPLE};

    List<LynxModule> allHubs;
    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private ElapsedTime autoTimer;
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

    public Function<Pose, PathChain> pathFactoryTangent;
    public Function<Pose, PathChain> pathFactoryLinear;

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
    SequentialCommandGroup pulseIntakeOut() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEOUT_ROLLERSIN)),
                new WaitCommand(600),
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN))
        );
    }
    SequentialCommandGroup visionIntake() {
        //Wait for either to finish:
        return new SequentialCommandGroup(
                new ParallelRaceGroup(
                    //1. Intake forever
                    new RepeatCommand(
                            new SequentialCommandGroup(
                                    //Scan at low post
                                    new DeferredCommand(() -> new ScanAndDriveToBallCommand(follower, limelight, true))
                                            .raceWith(new WaitUntilCommand(follower::isRobotStuck)),
                                    setCount(2),
                                    //Drive to hp zone
                                    new FollowPathCommand(follower, pathFactoryTangent.apply(Paths.Poses.INTAKE_HP)),
                                    new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.INTAKE_HP2)),
                                    setCount(3),
                                    //Scan at low pos
                                    new DeferredCommand(() -> new ScanAndDriveToBallCommand(follower, limelight, true))
                                            .raceWith(new WaitUntilCommand(follower::isRobotStuck)),
                                    setCount(4),
                                    //Drive back to med pos
                                    new DeferredCommand(() -> new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.SCAN_MED))),
                                    setCount(5),
                                    //Scan at low post
                                    new DeferredCommand(() -> new ScanAndDriveToBallCommand(follower, limelight, true))
                                            .raceWith(new WaitUntilCommand(follower::isRobotStuck)),
                                    setCount(2),
                                    //Drive to hp zone
                                    new FollowPathCommand(follower, pathFactoryTangent.apply(Paths.Poses.INTAKE_HP)),
                                    new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.INTAKE_HP2)),
                                    setCount(3),
                                    //Scan at low pos
                                    new DeferredCommand(() -> new ScanAndDriveToBallCommand(follower, limelight, true))
                                            .raceWith(new WaitUntilCommand(follower::isRobotStuck)),
                                    setCount(4),
                                    //Drive back to med pos
                                    new DeferredCommand(() -> new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.LAUNCH))),
                                    setCount(5)
                            )
                    ),
                    //2. Intaken 3
                    new SequentialCommandGroup(
                            new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                            new WaitForColorCommand(colorsensor),
                            new WaitCommand(100),
                            new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                            new WaitCommand(400),
                            new WaitForColorCommand(colorsensor),
                            new WaitCommand(100),
                            new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                            new WaitCommand(400),
                            new WaitForColorCommand(colorsensor),
                            new WaitCommand(100)
                    )
                )
        );
    }
    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();
        autoTimer = new ElapsedTime();

        //systems and pedro
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
        limelight.setPipeline(LimelightSubsystem.LIMELIGHT_PIPELINES.ARTIFACT_ONLY);
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        colorsensor.updateBack();
        lastVoltageCheck.reset();
        Paths paths = new Paths(follower);
        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        pathFactoryTangent = pose -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setTangentHeadingInterpolation()
                .build();
        pathFactoryLinear = pose -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, pose.getHeading(), 0.1))
                .build();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(115);
        SequentialCommandGroup far_volume = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetTicks(1440);
                    gate.down();
                }),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    gate.down();
                    autoTimer.reset();
                }), //bruh
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootFarPreload, true)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN))))
                ),
                setCount(1),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
                new WaitCommand(500),
                setCount(2),
                shootFourTimesWithDelay(),
                setCount(3),

                //Third row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRowFar)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootThirdRowFar, true),
                new WaitCommand(500), //to prevent moving while shooting
                shootFourTimesWithDelay(),

                //HP row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, paths.intakeHp1)
                                        .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                        .withTimeout(3000),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, paths.intakeHpBack),
                                new FollowPathCommand(follower, paths.intakeHp2)
                                        .withTimeout(3000)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                                new WaitForColorCommand(colorsensor),
                                new WaitCommand(100),
                                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                                new WaitCommand(400),
                                new WaitForColorCommand(colorsensor),
                                new WaitCommand(100),
                                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                                new WaitCommand(400),
                                new WaitForColorCommand(colorsensor),
                                new WaitCommand(100)
                        )
                ),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new WaitCommand(200),
                new FollowPathCommand(follower, paths.shootHpFar, true)
                        .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.2).andThen(pulseIntakeOut())),
                new WaitCommand(500), //to prevent moving while shooting
                new InstantCommand(gate::up),
                new WaitCommand(200),
                //Sort
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, true),
                new WaitCommand(400),
                new InstantCommand(gate::down),
                new WaitCommand(200),
                shootFourTimesWithDelay(),

                //VISION CYCLE 1
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, paths.toScanLow),
                        visionIntake().withTimeout(7000),
                        new DeferredCommand(() -> new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.LAUNCH))),
                        new WaitCommand(500),
                        shootFourTimesWithDelay()
                ),

                //VISION CYCLE 2
                setCount(10),
                new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.SCAN_LOW)),
                visionIntake()
                        .raceWith(new WaitUntilCommand(() -> autoTimer.seconds() > 24)),
                new DeferredCommand(() -> new FollowPathCommand(follower, pathFactoryLinear.apply(Paths.Poses.LAUNCH))),
                new WaitCommand(500),
                shootFourTimesWithDelay(),


                //move to end pos
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.parkFar)
        );

        schedule(
                new RunCommand(() -> follower.update())
        );
        schedule(new SequentialCommandGroup(
                    far_volume
            ));


    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
//        colorsensor.updateBack();
        if (shooter.getVelocityTicks() - shooter.getTargetTicks() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getVelocityTicks() - shooter.getTargetTicks() > 50) {
            led.setColor(LEDSubsystem.LEDState.BLUE);
        }
        else {
            led.setColor(LEDSubsystem.LEDState.GREEN);
        }
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
        }


        telemetry.addData("Loop Time", timer.milliseconds());

        telemetry.addData("Ball ty", limelight.getResult().getTy());
        telemetry.addData("Ball ta", limelight.getResult().getTa());
        telemetry.addData("Ball tyNC", limelight.getResult().getTyNC());

        telemetry.addData("Debug Counter", debug);
        telemetry.addData("Detected Motif", Arrays.toString(motif));

//        telemetry.addData("spindexer output", spindexer.getOutput());
//        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
//        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("spindexer's balls", Arrays.toString(spindexer.getBalls()));

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetTicks());
        telemetry.addData("shooter actual velocity", shooter.getVelocityTicks());

        telemetry.addData("------------------",null);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",null);
        currentPose = follower.getPose().plus(
                new Pose(-2,0) //DO NOT MIRROR THIS! INVERT THE X AXIS *ONLY*
        ); //Auto->teleop offset
        AutoPoseSaver.lastPose = currentPose;
        timer.reset();
        telemetry.update();
        super.run();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

    }

    @Override
    public void end() {
        AutoPoseSaver.lastPose = currentPose;
        super.end();
    }
}
