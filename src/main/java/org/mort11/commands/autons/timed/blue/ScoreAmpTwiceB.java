// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.autons.timed.blue;

import org.mort11.commands.actions.drivetrain.MoveToAprilTag;
import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.IntakeToVelocity;
import org.mort11.commands.actions.endeffector.armwrist.ArmToPos;
import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;
import org.mort11.configuration.constants.PhysicalConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ScoreAmpTwiceB extends SequentialCommandGroup {
  /** Creates a new TwoPiece. */
  public ScoreAmpTwiceB() {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ScoreAmpB(),
    SetArmAndWristPos.intake().withTimeout(.5),
    new TimedDrive(1,0,1.35, 0),
    new WaitCommand(0.5),
      new ParallelCommandGroup(
       new TimedDrive(0.65,1.2,0,0),
       new IntakeToVelocity(Intake.INTAKE_SPEED).withTimeout(0.7)
      ),
    SetArmAndWristPos.rest().withTimeout(.5),
    new TimedDrive(1,0,-0.9, 0),
    new TimedDrive(1,-1.4 ,0,0),
    SetArmAndWristPos.amp().withTimeout(1.5),
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
    // SetArmAndWristPos.amp().withTimeout(1.75),
    // new IntakeToVelocity(-0.5).withTimeout(0.75),
    // SetArmAndWristPos.rest().withTimeout(.5)
    // );
  }
}
