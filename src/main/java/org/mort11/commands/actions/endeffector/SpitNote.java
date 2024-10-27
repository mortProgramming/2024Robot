// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions.endeffector;

import static org.mort11.configuration.constants.PhysicalConstants.Wrist.WRIST_INTAKE_POS;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.WRIST_SPIT_POS;

import org.mort11.commands.actions.endeffector.armwrist.WristToPos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitNote extends SequentialCommandGroup {
  /** Creates a new SpitNote. */
  public SpitNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristToPos(WRIST_SPIT_POS).withTimeout(.5),
      new IntakeToVelocity(-1).withTimeout(.5),
      new WristToPos(WRIST_INTAKE_POS)
    );
  }
}
