// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.autons.timed.blue;

import org.mort11.commands.actions.drivetrain.Orient;
import org.mort11.commands.actions.drivetrain.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TaxiB extends SequentialCommandGroup {
  /** Move the robot forward, far enough to gain taxi points. */
  public TaxiB() {
    addCommands(
      new Orient(90),
      new TimedDrive(8, 0, 1, 0)
    );
  }
}
