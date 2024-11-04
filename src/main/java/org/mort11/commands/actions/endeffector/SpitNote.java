// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions.endeffector;

import org.mort11.commands.actions.endeffector.pos.WristToPos;
import org.mort11.commands.actions.endeffector.velocity.IntakeToVelocity;

import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.*;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SpitNote extends SequentialCommandGroup {
  public SpitNote() {
    addCommands(
      new WristToPos(WRIST_SPIT_POS).withTimeout(0.5),
      new IntakeToVelocity(SHOOTER_SHOOT_SPEED).withTimeout(0.5),
      new WristToPos(WRIST_INTAKE_POS)
    );
  }
}
