// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Orient extends SequentialCommandGroup {
  private Drivetrain drivetrain;

  public Orient(double angle) {
    drivetrain = Drivetrain.getInstance();
    addCommands(
      drivetrain.setGyroscopeZero(angle)
    );
  }
}
