package org.mort11.commands.autons.timed.red;

import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.commands.actions.drivetrain.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TaxiR extends SequentialCommandGroup {
  public TaxiR() {
    addCommands(
      new Orient(270),
      new TimedDrive(8, 0, 1, 0)
    );
  }
}
