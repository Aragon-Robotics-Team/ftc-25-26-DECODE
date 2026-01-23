package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class MoveSpindexerAndUpdateArrayCommand extends CommandBase {
    private SpindexerSubsystem spindexerSubsystem;
    private GateSubsystem gateSubsystem;
    public int number;
    private boolean instant = false;
    private final ElapsedTime safetyTimer = new ElapsedTime();
    private final ElapsedTime delayTimer = new ElapsedTime();
    private final double TIMEOUT_SEC = 2.5;
    public final double DELAY_SEC = 0.5;
    private boolean slow;
    private int timesMoved = 0;
    public MoveSpindexerAndUpdateArrayCommand(SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, int num, boolean instant, boolean slow) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.gateSubsystem = gateSubsystem;
        this.number = num;
        this.instant = instant;
        this.slow = slow;
    }

    @Override
    public void initialize() {
        safetyTimer.reset();
        delayTimer.reset();
        timesMoved = 0;
        if (!slow) {
            spindexerSubsystem.moveSpindexerBy(number * 120);
            if (gateSubsystem.gateState == GateSubsystem.GateState.DOWN) {
                spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
            }
            spindexerSubsystem.shiftBallsArrayBy(number);
            timesMoved += (int) Math.signum(number);
        }
    }

    @Override
    public void execute() {
        if (slow && timesMoved < number) {
            if (delayTimer.seconds() > DELAY_SEC) {
                spindexerSubsystem.moveSpindexerBy(120);
                if (gateSubsystem.gateState == GateSubsystem.GateState.DOWN) {
                    spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
                }
                spindexerSubsystem.shiftBallsArrayBy(1);
                timesMoved++;
                delayTimer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() { //finishes if timer runs out or at target position
        if (instant) {
            return true;
        }
        if (slow && timesMoved < number) {
            return false;
        }
        boolean atTarget = spindexerSubsystem.isNearTargetPosition();

        boolean timedOut = safetyTimer.seconds() > TIMEOUT_SEC;

        return atTarget || timedOut;
    }
}
