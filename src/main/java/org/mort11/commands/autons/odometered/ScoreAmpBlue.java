package org.mort11.commands.autons.odometered;

import org.mort11.commands.actions.drivetrain.DriveToPos;
import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.config.Odometer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpBlue extends SequentialCommandGroup{
    public ScoreAmpBlue() {
        addCommands( 
            new SequentialCommandGroup(
                new Orient(270),
                Odometer.resetOdometryCommand(0.4, 7.4, 270),
                new DriveToPos(1.5, 7.9)
            )
        );
    }
}
