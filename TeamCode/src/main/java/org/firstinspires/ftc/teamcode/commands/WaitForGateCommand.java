package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;

public class WaitForGateCommand extends CommandBase {
    private final GateSubsystem gateSubsystem;
    private final ElapsedTime safetyTimer = new ElapsedTime();
    public WaitForGateCommand(GateSubsystem gateSubsystem) {
        this.gateSubsystem = gateSubsystem;
    }

    @Override
    public void initialize() {
        safetyTimer.reset();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return gateSubsystem.isAtTarget() || safetyTimer.seconds() > 0.3;
    }
}
