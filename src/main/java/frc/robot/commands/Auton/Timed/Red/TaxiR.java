package frc.robot.commands.Auton.Timed.Red;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.RobotStart;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;

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
