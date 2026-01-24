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
@Autonomous(name = "\uD83D\uDD35 9 Sorted with Gate Intake", group = "angryBirds", preselectTeleOp = "BlueTeleOp")
public class Blue9SortClassifyAuto extends CommandOpMode {
    public static class Paths {
        //close autos
        public PathChain shootClosePreload;
        public PathChain intakeSecondRowClose;
        public PathChain shootSecondRowClose;
        public PathChain intakeThirdRowClose;
        public PathChain shootThirdRowClose;
        public PathChain intakeFirstRowClose;
        public PathChain shootFirstRowClose;
        public PathChain intakeRamp;
        public PathChain shootRamp;
        public PathChain parkAfter12Overflow;
        public PathChain parkAfter12Hold;
        public PathChain parkAfterShoot;

        //far autos
        public PathChain shootFarPreload;
        public PathChain intakeThirdRowFar;
        public PathChain shootThirdRowFar;
        public PathChain parkFar;

        public Paths(Follower follower) {
            shootClosePreload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21, 127), new Pose(52.6, 86.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(132))
                    .build();
            intakeSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.6, 86.800),
                                    new Pose(66, 30.000),
                                    new Pose(8, 47.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            shootSecondRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(8, 47.000),
                                    new Pose(66, 55.000),
                                    new Pose(55.6, 81.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                    .build();

            intakeThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.6, 81.800),
                                    new Pose(63.7, 19.1),
                                    new Pose(12.3, 21.6)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(180))
                    .build();

            shootThirdRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(12.3, 21.6), new Pose(55.6, 81.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();

            intakeFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.6, 81.800), new Pose(29, 77.000))
                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .setTangentHeadingInterpolation()
                    .build();

            shootFirstRowClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(29, 77.000), new Pose(55.6, 81.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(133))
                    .build();
            intakeRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.6, 81.800),
                                    new Pose(55, 58.000),
                                    new Pose(12.3, 49)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(105))
                    .build();

            shootRamp = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.3, 49),
                                    new Pose(55, 58.000),
                                    new Pose(55.6, 81.800)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(133))
                    .build();

            parkAfter12Overflow = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.6, 81.800), new Pose(39, 83))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(270))
                    .build();
            parkAfter12Hold = follower
                    .pathBuilder()
                    .addPath(
//                            new BezierLine(new Pose(131.700, 21.6), new Pose(105, 83))
                            new BezierCurve(
                                    new Pose(12.3, 21.6),
                                    new Pose(90, 52.3),
                                    new Pose(39, 83)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            parkAfterShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.6, 81.800), new Pose(39,83))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
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
    final AUTOS CURRENTAUTO = AUTOS.NINE_INTAKEGATE_HOLD;

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
    public static Pose startingPose = new Pose(21,127,Math.toRadians(0));
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
        SequentialCommandGroup nine_sorted = new SequentialCommandGroup(
                new InstantCommand(() -> { //setup
                    shooter.setTargetTicks(1150);
                    gate.down();
                    spindexer.setBalls(new RobotConstants.BallColors[] {GREEN, PURPLE, PURPLE});
                }),
                //Preload
                new ParallelDeadlineGroup(
                        new FollowPathCommand(follower, paths.shootClosePreload, false)
                                .alongWith(new WaitUntilCommand(() -> follower.getPathCompletion() > 0.1).andThen(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))),
                        new WaitUntilCommand(() -> follower.getPathCompletion() > 0.6).andThen(new InstantCommand(this::scanMotif))
                ).alongWith(new ParallelCommandGroup(
                        new WaitCommand(100),
                        new InstantCommand(gate::up),
                        new WaitCommand(100),
                        new InstantCommand(gate::down)
                )),
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
                new FollowPathCommand(follower, paths.shootSecondRowClose, false),
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
                new FollowPathCommand(follower, paths.shootFirstRowClose),
                new DeferredCommand(() -> new ShootSortedBallsCommandSequence(shooter, spindexer, gate, intake, motif))
        );
        SequentialCommandGroup intake_gate_shoot_and_park = new SequentialCommandGroup(
                //move to end pos
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, paths.intakeRamp, 1.0).withTimeout(3000)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)))
                                .withTimeout(3000),
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
                new FollowPathCommand(follower, paths.shootThirdRowClose),
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
                new FollowPathCommand(follower, paths.parkAfter12Hold, 1.0).alongWith(new InstantCommand(() -> {gate.up();}))
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
        timer.reset();
        telemetry.update();
        super.run();

    }

    @Override
    public void end() {
        blackboard.put("endpose", currentPose);
        super.end();
    }
}
