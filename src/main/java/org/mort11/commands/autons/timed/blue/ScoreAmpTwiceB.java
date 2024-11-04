// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.autons.timed.blue;

import org.mort11.commands.actions.drivetrain.TimedDrive;
import org.mort11.commands.actions.endeffector.pos.SetArmAndWristPos;
import org.mort11.commands.actions.endeffector.velocity.IntakeToVelocity;
import org.mort11.configuration.constants.PhysicalConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ScoreAmpTwiceB extends SequentialCommandGroup {
  public ScoreAmpTwiceB() {
    
    addCommands(
      new ScoreAmpB(),
      SetArmAndWristPos.intake().withTimeout(0.5),
      new TimedDrive(1, 0, 1.35,  0),
      new WaitCommand(0.5),
        new ParallelCommandGroup(
        new TimedDrive(0.65,1.2,0,0),
        new IntakeToVelocity(Intake.INTAKE_SPEED).withTimeout(0.7)
      ),
      SetArmAndWristPos.rest().withTimeout(0.5),
      new TimedDrive(1,0,-0.9, 0),
      new TimedDrive(1,-1.4 ,0,0),
      SetArmAndWristPos.amp().withTimeout(1.5),
      new IntakeToVelocity(-0.5).withTimeout(0.75),
      SetArmAndWristPos.rest().withTimeout(0.5)
    );
  }
}
