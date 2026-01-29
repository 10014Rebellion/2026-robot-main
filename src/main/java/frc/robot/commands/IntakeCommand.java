package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.IntakeRoller.IntakeRollerSubsystem;

public class IntakeCommand extends Command {
    private final IntakeRollerSubsystem intakeRoller;
    
    public IntakeCommand(IntakeRollerSubsystem intakeRoller) {
        this.intakeRoller = intakeRoller;
        addRequirements(intakeRoller);
    }
    
    @Override
    public void execute() {
        intakeRoller.intake();
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRoller.stopRollers();
    }
}