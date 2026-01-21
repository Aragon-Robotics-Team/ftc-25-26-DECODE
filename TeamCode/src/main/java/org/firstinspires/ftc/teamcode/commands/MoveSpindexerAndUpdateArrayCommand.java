package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class MoveSpindexerAndUpdateArrayCommand extends CommandBase {
    private SpindexerSubsystem spindexerSubsystem;
    private GateSubsystem gateSubsystem;
    public int number;
    private boolean instant = false;
    private final ElapsedTime safetyTimer = new ElapsedTime();
    private final double TIMEOUT_SEC = 2.5;
    public MoveSpindexerAndUpdateArrayCommand(SpindexerSubsystem spindexerSubsystem, GateSubsystem gateSubsystem, int num, boolean instant) {
        this.spindexerSubsystem = spindexerSubsystem;
        this.gateSubsystem = gateSubsystem;
        number = num;
        this.instant = instant;
    }

    @Override
    public void initialize() {
        safetyTimer.reset();
        if (gateSubsystem.gateState == GateSubsystem.GateState.DOWN) {
            spindexerSubsystem.setBallAt(2, RobotConstants.BallColors.NONE);
        }
        spindexerSubsystem.moveSpindexerBy(120 * number);
        spindexerSubsystem.shiftBallsArrayBy(number);
    }

    @Override
    public boolean isFinished() { //finishes if timer runs out or at target position
        if (instant) {
            return true;
        }
        boolean atTarget = spindexerSubsystem.isNearTargetPosition();

        boolean timedOut = safetyTimer.seconds() > TIMEOUT_SEC;

        return atTarget || timedOut;
    }
}
