// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.Timed.Blue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Actions.Drivetrain.MoveToAprilTag;
import frc.robot.commands.Actions.Drivetrain.TimedDrive;
import frc.robot.commands.Actions.EndEffector.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.utility.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;


public class ScoreAmpTwiceB extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */
  public ScoreAmpTwiceB() {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ScoreAmpB(),
    new TimedDrive(1,0,1.3, 0),
    SetArmAndWristPos.intake().withTimeout(.5),
      new ParallelCommandGroup(
      new TimedDrive(0.65,1,0,0),
      new IntakeToVelocity(Constants.Intake.INTAKE_SPEED).withTimeout(0.7)
    ),
    SetArmAndWristPos.rest().withTimeout(.5),
    new TimedDrive(1,0,-1.3, 0),
    new TimedDrive(0.75,-1.1,0,0),
    SetArmAndWristPos.score().withTimeout(1.75),
    new IntakeToVelocity(-0.5).withTimeout(0.75),
    SetArmAndWristPos.rest().withTimeout(.5)
    );
    // addCommands(    new ScoreAmpB(),
    // new TimedDrive(true, 1,0,1.3, 0),
    // SetArmAndWristPos.intake().withTimeout(.5),
    //   new ParallelCommandGroup(
    //   new TimedDrive(true, 0.65,1,0,0),
    //   new IntakeToVelocity(Constants.Intake.INTAKE_SPEED).withTimeout(0.7)
    // ),
    // SetArmAndWristPos.rest().withTimeout(.5),
    // new TimedDrive(true, 1,0,-1.3, 0),
    // new TimedDrive(true, 0.75,-1.1,0,0),
    // SetArmAndWristPos.score().withTimeout(1.75),
    // new IntakeToVelocity(-0.5).withTimeout(0.75),
    // SetArmAndWristPos.rest().withTimeout(.5)
    // );
  }
}
