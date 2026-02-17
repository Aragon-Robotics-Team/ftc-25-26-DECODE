package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.MoveSpindexerAndUpdateArrayCommand;
import org.firstinspires.ftc.teamcode.opmodes.FunnyNeuralNetworkTeleop;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class SubsystemTuningOp extends CommandOpMode {
    //Gamepad
    GamepadEx driver1;

    //Subsystem
    SpindexerSubsystem spindexer;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;

    //FTCDashboard constants
    static double shooter_kP;
    static double shooter_kF;
    static double spindexer_kP;
    static double spindexer_kD;

    static double shooter_hoodPos = 0.5;
    static double shooter_linearSpeed = 0;

    @Override
    public void initialize() {
        spindexer = new SpindexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        driver1 = new GamepadEx(gamepad1);

        shooter_kP = shooter.kPOriginal;
        shooter_kF = shooter.kFOriginal;
        spindexer_kP = spindexer.kP;
        spindexer_kD = spindexer.kD;

        if (Math.abs(spindexer.getWrappedPosition() - 115) < 60) {
            spindexer.set(115);
        }
        else if (Math.abs(spindexer.getWrappedPosition() - 235) < 60){
            spindexer.set(235);
        }
        else if (Math.abs(spindexer.getWrappedPosition() - 355) < 60) {
            spindexer.set(355);
        }
        else {
            spindexer.set(115);
        }

        register(spindexer, shooter, intake);

        driver1.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                () -> spindexer.moveSpindexerBy(-120)
        );
        driver1.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                () -> spindexer.moveSpindexerBy(120)
        );
        driver1.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(
                    () -> intake.set(IntakeSubsystem.IntakeState.INTAKEIN_ROLLERSIN)
                )
                .whenReleased(
                    () -> intake.set(IntakeSubsystem.IntakeState.INTAKESTILL_ROLLERSSTILL)
                )
        ;
    }
    @Override
    public void run() {
        shooter.setPIDF(shooter_kP, 0, 0, shooter_kF);
        spindexer.setPIDCoefficients(spindexer_kP, 0, spindexer_kD, 0);
        shooter.setTargetLinearSpeed(shooter_linearSpeed);
        shooter.setHoodPos(shooter_hoodPos); //Temporary - comment out shooter periodic() hood.set if u want to use this
        super.run();
    }
}
