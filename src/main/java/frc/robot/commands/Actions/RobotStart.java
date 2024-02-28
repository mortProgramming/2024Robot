// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Odometer;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotStart extends SequentialCommandGroup {
  /** Creates a new RobotStart. */
  private Odometer odometer;
  private Drivetrain drivetrain;
  public RobotStart(double angle) {
    drivetrain = Drivetrain.getInstance();
    odometer = Odometer.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05),
      new InstantCommand(() -> odometer.resetOdometry())

    );
  }

  public RobotStart(double x, double y, double angle) {
    drivetrain = Drivetrain.getInstance();
    odometer = Odometer.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05),
      new InstantCommand(() -> odometer.resetOdometry(new Pose2d(x, y, new Rotation2d(angle))))

    );
  }
}
