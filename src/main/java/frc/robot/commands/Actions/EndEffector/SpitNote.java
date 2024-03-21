// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Wrist.WRIST_INTAKE_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_SPIT_POSITION;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.EndEffector.ArmWrist.WristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpitNote extends SequentialCommandGroup {
  /** Creates a new SpitNote. */
  public SpitNote() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristToPosition(WRIST_SPIT_POSITION).withTimeout(.5),
      new IntakeToVelocity(-1).withTimeout(.5),
      new WristToPosition(WRIST_INTAKE_POSITION)
    );
  }
}
