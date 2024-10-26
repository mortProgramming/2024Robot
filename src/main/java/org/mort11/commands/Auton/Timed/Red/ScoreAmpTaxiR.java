// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.Auton.Timed.Red;

import org.mort11.commands.Actions.RobotStart;
import org.mort11.commands.Actions.Drivetrain.TimedDrive;
import org.mort11.commands.Auton.Timed.Blue.ScoreAmpB;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAmpTaxiR extends SequentialCommandGroup {
  /** Creates a new ScoreAmpRTaxi. */
  public ScoreAmpTaxiR() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreAmpR(),
      // new TimedDrive(true, 6.5, 0, 1, 270)
      new TimedDrive(6.5, 0, 1, 0)

    );
  }
}
