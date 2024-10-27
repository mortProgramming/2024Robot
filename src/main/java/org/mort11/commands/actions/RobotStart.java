// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions;

import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;
import org.mort11.configuration.Odometer;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotStart extends SequentialCommandGroup {
  /** Creates a new RobotStart. */
  private Drivetrain drivetrain;

  public RobotStart(double angle) {
    drivetrain = Drivetrain.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05)
    );
  }

  public RobotStart(boolean isBlue, double angle) {
    drivetrain = Drivetrain.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05),
      new InstantCommand(() -> Odometer.resetOdometry()),
      new InstantCommand(() -> drivetrain.setIsBlue(isBlue))

    );
  }

  public RobotStart(double x, double y, double angle) {
    drivetrain = Drivetrain.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05),
      new InstantCommand(() -> Odometer.resetOdometry(new Pose2d(x, y, new Rotation2d(angle))))

    );
  }

  public RobotStart(boolean isBlue, double x, double y, double angle) {
    drivetrain = Drivetrain.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.zeroGyroscope(angle)),
      SetArmAndWristPos.rest().withTimeout(0.05),
      new InstantCommand(() -> Odometer.resetOdometry(new Pose2d(x, y, new Rotation2d(angle)))),
      new InstantCommand(() -> drivetrain.setIsBlue(isBlue))
    );
  }
}
