package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.GREEN;
import static org.firstinspires.ftc.teamcode.RobotConstants.BallColors.PURPLE;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelDeadlineGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
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
@Autonomous(name = "\uD83D\uDD34 Far Volume", group = "angryBirds", preselectTeleOp = "RedTeleOp")
public class RedFarAuto extends CommandOpMode {
    //3 sorted preload, 6 sorted spike mark, gate intake
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
        public static class Poses {
            public static final Pose LAUNCH = new Pose(89.4,18.5, Math.toRadians(60.5));
            public static final Pose START = new Pose(102.5, 8, Math.toRadians(90));
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
                                    new Pose(126.13, 33.82)
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
                            new BezierLine(Poses.LAUNCH, new Pose(136, 8))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();

            intakeHpBack = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(136, 8), new Pose(121,8))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeHp2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(121,8), new Pose(136, 8))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootHpFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(136, 8),
                                    new Pose(118.5,30),
                                    Poses.LAUNCH
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0),Poses.LAUNCH.getHeading())
                    .build();

            intakeOverflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    Poses.LAUNCH,
                                    new Pose(144, 0),
                                    new Pose(144, 0),
                                    new Pose(130, 20),
                                    new Pose(136.3,54)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            shootOverflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130,45),Poses.LAUNCH)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Poses.LAUNCH.getHeading())
                    .build();

            parkFar = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(Poses.LAUNCH, new Pose(110, 8))
                    )
                    .setTangentHeadingInterpolation()
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
                new WaitCommand(100),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(300),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                new WaitCommand(100),
                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false)
        );
    }
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
        SequentialCommandGroup far_sorted = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetTicks(1450);
                    gate.down();
                    gate.down();
                }),
                new WaitCommand(1),
                new InstantCommand(() -> gate.down()), //bruh
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootFarPreload, true)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))),
                        new InstantCommand(this::scanMotif)
                ),
                setCount(1),
                new WaitUntilCommand(() -> shooter.isAtTargetVelocity()),
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
                new WaitCommand(300), //to prevent moving while shooting
                shootFourTimesWithDelay(),

                //HP row
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelRaceGroup(
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, paths.intakeHp1)
                                        .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                        .withTimeout(3000),

                                new FollowPathCommand(follower, paths.intakeHpBack),
                                new FollowPathCommand(follower, paths.intakeHp2)
                                        .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                        .withTimeout(3000)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)),
                                new WaitForColorCommand(colorsensor),
                                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                                new WaitCommand(400),
                                new WaitForColorCommand(colorsensor),
                                new MoveSpindexerAndUpdateArrayCommand(spindexer, gate, 1, true, false),
                                new WaitCommand(400),
                                new WaitForColorCommand(colorsensor)
                        )
                ),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootHpFar, true),
                new WaitCommand(300), //to prevent moving while shooting
                shootFourTimesWithDelay(),

                //Overflow
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeOverflow)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN))),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootOverflow, true),
                new WaitCommand(300), //to prevent moving while shooting
                shootFourTimesWithDelay(),

                //Overflow x2
                new InstantCommand(() -> follower.setMaxPower(0.8)),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeOverflow)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN))),
                        intakeArtifacts()
                ),
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.shootOverflow, true),
                new WaitCommand(300), //to prevent moving while shooting
                shootFourTimesWithDelay(),

                //move to end pos
                new InstantCommand(() -> follower.setMaxPower(1.0)),
                new FollowPathCommand(follower, paths.parkFar).withTimeout(3000)
        );

        schedule(
                new RunCommand(() -> follower.update())
        );
        schedule(new SequentialCommandGroup(
                    far_sorted
            ));


    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        colorsensor.updateSensor1();
        colorsensor.updateSensor2();
        colorsensor.updateBack();
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

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetTicks());
        telemetry.addData("shooter actual velocity", shooter.getVelocityTicks());

        telemetry.addData("------------------",null);

        telemetry.addData("current pos", String.format("X: %8.2f, Y: %8.2f", follower.getPose().getX(), follower.getPose().getY()));
        telemetry.addData("current heading", String.format("Heading: %.4f", follower.getPose().getHeading()));
        telemetry.addData("t value", follower.getCurrentTValue());
        telemetry.addData("------------------",null);
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
