package org.mort11.commands.Auton.Timed.Red;

import org.mort11.commands.Actions.RobotStart;
import org.mort11.commands.Actions.Drivetrain.TimedDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TaxiR extends SequentialCommandGroup {
  /** Move the robot forward, far enough to gain taxi points. */
  public TaxiR() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new RobotStart(90),
      new RobotStart(270),
      new TimedDrive(8, 0, 1, 0)
    );
  }
}
