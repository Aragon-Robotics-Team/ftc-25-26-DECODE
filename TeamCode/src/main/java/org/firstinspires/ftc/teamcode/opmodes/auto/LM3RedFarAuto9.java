package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.commands.MoveSpindexerCommand;
import org.firstinspires.ftc.teamcode.commands.WaitForColorCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorsSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;


@Config
@Autonomous(name = "Red 12 Ball Far", group = "angryBirds", preselectTeleOp = "Teleop")
public class LM3RedFarAuto9 extends CommandOpMode {
    //paths has moved to RedFarAutoPaths
    RedFarAutoPaths paths;
    public Pose currentPose;

    //voltage compensation
    public VoltageSensor voltageSensor;
    double currentVoltage = 14;
    private boolean slowMode = false;
    public ElapsedTime lastVoltageCheck = new ElapsedTime();
    private ElapsedTime timer;
    private Follower follower;

    //update starting pose
    public static Pose startingPose = new Pose(88.5,8.45, Math.toRadians(90.00));
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private SpindexerSubsystem spindexer;
    private ColorSensorsSubsystem colorsensor;
    private GateSubsystem gate;
    private LEDSubsystem led;
    public void buildPaths(Follower follower) {
        follower.setStartingPose(startingPose);
        paths = new RedFarAutoPaths(follower);
    }

   /* //preset commands
    private SequentialCommandGroup intakeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING)),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(1500)
                ),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                ),
                new MoveSpindexerCommand(spindexer, gate, 1, true),
                new ParallelRaceGroup(
                        new WaitForColorCommand(colorsensor),
                        new WaitCommand(500)
                )
        );
    }*/


    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        //systems and pedro
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        colorsensor = new ColorSensorsSubsystem(hardwareMap);
        gate = new GateSubsystem(hardwareMap);
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        lastVoltageCheck.reset();
        led = new LEDSubsystem(hardwareMap);


        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        //Idk what this is but I think it's important it was from the github code
        super.reset();

        // Initialize subsystems
        register(intake, spindexer, shooter, colorsensor, led, gate);
        spindexer.set(75);
        shooter.setHood(0.45);
        gate.down();

        //init paths
        buildPaths(follower);


        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> follower.update()),
                new SequentialCommandGroup(
                        new InstantCommand(() -> { //immediately set shooter to max speed.
                            shooter.setTargetVelocity(1500);
                        }),
                        new FollowPathCommand(follower, paths.toSZ, true),
                        new WaitCommand(1000), //buffer time when testing but removeable for later
                        new InstantCommand(() -> { //launch all 3 balls
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                        }),
                        new WaitCommand(1000), //buffer time when testing but removeable for later

                        //FOR REFERENCE: SZ is Shooting Zone, C# is Cycle #
                        //toC# is moving to line up to intake balls in that cycle
                        //toShootingAngle is in place of a turn command, which can definitely be updated later
                        //THERE IS NO SORTING ON THE SHOOTING

                        //cycle 1
                        new FollowPathCommand(follower, paths.toC1)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING))),
                        //this parallelrace group can also be replaced with .raceWith I think but it's more clear to write it like this
                        new ParallelRaceGroup( //Do both, end when a or b finishes first:
                                new FollowPathCommand(follower, paths.intakeC1, 0.5)
                                        .withTimeout(3000), //a. intake path finishes
                                new SequentialCommandGroup( //b. the ball intaking sequence finishes
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor)
                                )
                        ),
                        new FollowPathCommand(follower, paths.returnToSZC1)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.STILL))),
                        new FollowPathCommand(follower, paths.toShootingAngle1, true),
                        new WaitCommand(1000), //buffer time when testing but removeable for later
                        new InstantCommand(() -> { //launch all 3 balls
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                        }),
                        new WaitCommand(1000), //buffer time when testing but removeable for later

                        //cycle 2
                        new FollowPathCommand(follower, paths.toC2)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.INTAKING))),
                        //this parallelrace group can also be replaced with .raceWith I think but it's more clear to write it like this
                        new ParallelRaceGroup( //Do both, end when a or b finishes first:
                                new FollowPathCommand(follower, paths.intakeC2, 0.5)
                                        .withTimeout(3000), //a. intake path finishes
                                new SequentialCommandGroup( //b. the ball intaking sequence finishes
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor),
                                        new MoveSpindexerCommand(spindexer, gate, 1, true),
                                        new WaitForColorCommand(colorsensor)
                                )
                        ),
                        new FollowPathCommand(follower, paths.adjustForWallC2)
                                .alongWith(new InstantCommand(() -> intake.set(IntakeSubsystem.IntakeState.STILL))),
                        new FollowPathCommand(follower, paths.returnToSZC2),
                        new FollowPathCommand(follower, paths.toShootingAngle2, true),
                        new WaitCommand(1000), //buffer time when testing but removeable for later
                        new InstantCommand(() -> { //launch all 3 balls
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                            spindexer.moveSpindexerBy(120);
                        }),
                        new WaitCommand(1000), //buffer time when testing but removeable for later

                        new FollowPathCommand(follower, paths.moveOffLine) // should go to the end point regardless of positions

                )

        );

    }
    @SuppressLint("DefaultLocale")
    @Override
    public void run() {
        if (shooter.getActualVelocity() - shooter.getTargetVelocity() < -30) {
            led.setColor(LEDSubsystem.LEDState.RED);
        }
        else if (shooter.getActualVelocity() - shooter.getTargetVelocity() > 50) {
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

        telemetry.addData("spindexer output", spindexer.getOutput());
        telemetry.addData("spindexer setpoint", spindexer.getPIDSetpoint());
        telemetry.addData("spindexer pos", spindexer.getCurrentPosition());
        telemetry.addData("is spindexer ready to read color ", spindexer.availableToSenseColor());
        telemetry.addData("spindexer's balls", spindexer.getBalls());

        telemetry.addData("------------------",null);

        telemetry.addData("shooter target velocity", shooter.getTargetVelocity());
        telemetry.addData("shooter actual velocity", shooter.getActualVelocity());

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
