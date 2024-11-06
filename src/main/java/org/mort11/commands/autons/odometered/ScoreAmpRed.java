package org.mort11.commands.autons.odometered;

import org.mort11.commands.actions.drivetrain.DriveToPos;
import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.config.Odometer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpRed extends SequentialCommandGroup{
    public ScoreAmpRed() {
        addCommands( 
            new SequentialCommandGroup(
                new Orient(270),
                Odometer.resetOdometryCommand(15.1, 7.4, 270),
                new DriveToPos(13, 7.4)
            )
        );
    }
}
