package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.RobotConstants.SPINDEXER_TICKS_PER_DEG;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class MoveSpindexerCommand extends CommandBase {
    private SpindexerSubsystem spindexerSubsystem;
    private GateSubsystem gateSubsystem;
    private IntakeSubsystem intakeSubsystem;
    public int number;
    private boolean instant = false;
    public MoveSpindexerCommand(SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, IntakeSubsystem intakeSubsystem, int num, boolean instant) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.gateSubsystem = gateSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        number = num;
        this.instant = instant;
    }

    @Override
    public void initialize() {
//        if (gateSubsystem.isAtTarget() && gateSubsystem.gateState == GateSubsystem.GateState.DOWN) {
//            spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
//        }
        spindexerSubsystem.moveSpindexerBy(120 * number);
//        spindexerSubsystem.shiftBallsArrayBy(number);
        if (number < 0) {
            intakeSubsystem.set(IntakeSubsystem.IntakeState.INTAKEREVERSE);
        }
    }

    @Override
    public boolean isFinished() {
        if (instant) {
            return true;
        }
        return spindexerSubsystem.isNearTargetPosition();
    }
}
