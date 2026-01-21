package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class WaitForShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    public WaitForShooterCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return (shooterSubsystem.getVelocityTicks() - shooterSubsystem.getTargetTicks() < -50) && (shooterSubsystem.getVelocityTicks() - shooterSubsystem.getTargetTicks() > 50);
    }
}
