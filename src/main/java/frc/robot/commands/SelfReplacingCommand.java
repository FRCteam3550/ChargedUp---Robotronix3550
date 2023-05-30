package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;


public class SelfReplacingCommand extends CommandBase {
    private final Supplier<Command> m_makeReplacement;
    private Command m_replacement = null;

    public SelfReplacingCommand(Supplier<Command> makeReplacement, Subsystem... requirements) {
        m_makeReplacement = requireNonNullParam(makeReplacement, "makeReplacement", "SelfReplacingCommand");
        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_replacement = m_makeReplacement.get();
        m_replacement.initialize();
    }

    @Override
    public void execute() {
        m_replacement.execute();
    }

    @Override
    public boolean isFinished() {
        return m_replacement.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        m_replacement.end(interrupted);
    }
        
}
