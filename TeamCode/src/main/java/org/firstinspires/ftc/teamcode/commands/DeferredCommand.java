package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import java.util.function.Supplier;

public class DeferredCommand extends CommandBase {
    private final Supplier<CommandBase> commandSupplier;
    private CommandBase m_command;
    private boolean initialized = false;

    // We pass a function that CREATES the command later
    public DeferredCommand(Supplier<CommandBase> commandSupplier) {
        this.commandSupplier = commandSupplier;
    }

    @Override
    public void initialize() {
        // This is the magic line.
        // It runs "new ShootSortedBalls..." NOW, seeing the current balls.
        m_command = commandSupplier.get();
        m_command.initialize();
        initialized = true;
    }

    @Override
    public void execute() {
        if (initialized) {
            m_command.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return initialized && m_command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (initialized) {
            m_command.end(interrupted);
        }
    }
}