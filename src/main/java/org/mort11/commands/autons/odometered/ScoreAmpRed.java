package org.mort11.commands.autons.odometered;

import org.mort11.commands.actions.drivetrain.DriveToPos;
import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreAmpRed extends SequentialCommandGroup{
    public Drivetrain drivetrain;

    public ScoreAmpRed() {
        drivetrain = Drivetrain.getInstance();
        
        addCommands( 
            new SequentialCommandGroup(
                new Orient(270),
                new InstantCommand(
                    () -> drivetrain.getSwerveDrive().resetPosition(
                        new Pose2d(
                            0.4, 7.4, 
                            Rotation2d.fromDegrees(270)
                        )
                    ), drivetrain
                ),
                new DriveToPos(13, 7.4)
            )
        );
    }
}
