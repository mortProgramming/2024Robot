package org.mort11.library.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandUtils {

    public Command toCommand(Runnable runnable, Subsystem subsystem) {
        return new InstantCommand(runnable, subsystem);
    }

    public Trigger doubleInput (Trigger trigger1, Trigger trigger2) {
        return trigger1.and(trigger2);
    }
}
