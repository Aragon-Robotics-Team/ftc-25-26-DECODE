package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
@Autonomous(name = "\uD83D\uDD35 12 Sorted Overflow", group = "angryBirds", preselectTeleOp = "RedTeleOp")
public class Blue12OverflowSortedAuto extends CommandOpMode {
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
        public PathChain parkAfter12Overflow;
        public PathChain parkAfter12Hold;
        public PathChain parkAfterShoot;

        public static class Poses {
            public static final Pose LAUNCH = new Pose(144-93.3, 89.8, Math.toRadians(180-45));
            public static final Pose START = new Pose(144-129,115,Math.toRadians(180-180));
            public static final Pose PARK_CLOSE = new Pose(144-106,75.3, Math.toRadians(180+90));
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
                                    new Pose(144-87.6, 43),
                                    new Pose(144-126.13, 54)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-25), Math.toRadians(180-0))
                    .build();

//            hitGateSecond = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(126.13, 54), new Pose(126.13, 64))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
//                    .build();

            shootSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-126.13, 54),
                                    new Pose(144-87.6, 43),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Poses.LAUNCH.getHeading())
                    .build();

            intakeFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, new Pose(144-120.63, 85.6))
                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .setTangentHeadingInterpolation()
                    .build();

            shootFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-120.63, 85.6), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Poses.LAUNCH.getHeading())
                    .build();

            intakeThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(144-99, 12.75),
                                    new Pose(144-132.13, 31.6)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-25), Math.toRadians(180-0))
                    .build();

            shootThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-132.13, 31.6), Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Poses.LAUNCH.getHeading())
                    .build();

            intakeRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(144-86.4, 45.6),
                                    new Pose(144-139, 51.7)
                            )
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Math.toRadians(180-75))
                    .build();

            shootRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(144-139, 51.7),
                                    new Pose(144-86.4, 45.6),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-75), Poses.LAUNCH.getHeading())
                    .build();

            parkAfter12Overflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, Poses.PARK_CLOSE)
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Poses.PARK_CLOSE.getHeading())
                    .build();
            parkAfter12Hold = follower
                    .pathBuilder()
                    .addPath(
//                            new BezierLine(new Pose(131.700, 21.6), new Pose(105, 83))
                            new BezierCurve(
                                    new Pose(144-135.3, 36.6),
                                    new Pose(144-95, 35),
                                    Poses.PARK_CLOSE
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-0), Poses.PARK_CLOSE.getHeading())
                    .build();

            parkAfterShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, Poses.PARK_CLOSE)
                    )
                    .setLinearHeadingInterpolation(Poses.LAUNCH.getHeading(), Poses.PARK_CLOSE.getHeading())
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

    //Selectiopn
    private enum AUTOS {
        TWELVE_HOLD, TWELVE_OVERFLOW, NINE_INTAKEGATE_HOLD//, FAR
    }
    final AUTOS CURRENTAUTO = AUTOS.TWELVE_OVERFLOW;

    public Pose currentPose;
    public RobotConstants.BallColors[] motif = new RobotConstants.BallColors[]{PURPLE, PURPLE,PURPLE};

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
        SequentialCommandGroup nine_sorted = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetTicks(1140);
                    gate.down();
                    spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});
                }),
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootClosePreload, true)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6).andThen(new InstantCommand(this::scanMotif)),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.8).andThen(new InstantCommand(this::scanMotif))
                ),
                setCount(1),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
                setCount(2),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif)),
                setCount(3),

                //Second row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeSecondRowClose)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, GREEN, PURPLE});}),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
//                new FollowPathCommand(follower, paths.hitGateSecond).withTimeout(1500),
                new FollowPathCommand(follower, paths.shootSecondRowClose, true),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif)),

                //First row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeFirstRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});}),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootFirstRowClose, true),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif))
//                //Ramp cycle
//                new FollowPathCommand(follower, paths.intakeRamp, false),
//                intakeArtifacts().withTimeout(3000),
//                new FollowPathCommand(follower, paths.shootRamp, true),
//                new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif),
        );
        SequentialCommandGroup intake_gate_shoot_and_park = new SequentialCommandGroup(
                //move to end pos
                new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN);
                            shooter.setTargetTicks(600);
                        }),
                        new FollowPathCommand(follower, paths.intakeRamp, 1.0).withTimeout(3000).withTimeout(3000),
                        new WaitCommand(3000).andThen(
                                new SequentialCommandGroup(
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                                        new WaitCommand(500),
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false)
                                )
                        )
                )
        );
        SequentialCommandGroup park_overflow = new SequentialCommandGroup(
                //intake third row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        new WaitCommand(2000)
                                .andThen(intakeArtifacts())
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, PURPLE, GREEN});}),

                //shoot third row
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootThirdRowClose, true),
//                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif)),
                new DeferredCommand(() -> new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 4, false, false)),
                //move to end pos
                new FollowPathCommand(follower, paths.parkAfter12Overflow)
        );
        SequentialCommandGroup park_twelve = new SequentialCommandGroup(
                //intake third row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeThirdRowClose).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
                        new WaitCommand(2000)
                                .andThen(intakeArtifacts())
                ),
                new InstantCommand(() -> {spindexer.setBalls(new RobotConstants.BallColors[] {PURPLE, PURPLE, GREEN});}),

                //move to end pos
                new FollowPathCommand(follower, paths.parkAfter12Hold,true, 1.0).alongWith(new InstantCommand(() -> {gate.up();}))
        );

        schedule(
                new RunCommand(() -> follower.update())
        );

        if (CURRENTAUTO == AUTOS.TWELVE_OVERFLOW) {
            schedule(new SequentialCommandGroup(
                    nine_sorted,
                    park_overflow
            ));
        }
        else if (CURRENTAUTO == AUTOS.TWELVE_HOLD) {
            schedule(new SequentialCommandGroup(
                    nine_sorted,
                    park_twelve
            ));
        }
        else if (CURRENTAUTO == AUTOS.NINE_INTAKEGATE_HOLD) {
            schedule(new SequentialCommandGroup(
                    nine_sorted,
                    intake_gate_shoot_and_park
            ));
        }

    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
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
        //Voltage compensation code
        if (lastVoltageCheck.milliseconds() > 500) { //check every 500ms
            currentVoltage = voltageSensor.getVoltage();
            spindexer.updatePIDVoltage(currentVoltage);
            shooter.updatePIDVoltage(currentVoltage);
            lastVoltageCheck.reset();
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
}
