package org.mort11.commands.autons.odometered;

import org.mort11.commands.actions.drivetrain.MoveToPos;
import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.configuration.Odometer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpBlue extends SequentialCommandGroup{
    public ScoreAmpBlue() {
        addCommands( 
            new SequentialCommandGroup(
                new Orient(270),
                Odometer.resetOdometryCommand(0.4, 7.4, 270),
                new MoveToPos(1.5, 7.9)
            )
        );
    }
}
