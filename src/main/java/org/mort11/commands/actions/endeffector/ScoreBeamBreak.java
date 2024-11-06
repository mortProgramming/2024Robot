// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions.endeffector;

import static org.mort11.config.constants.PhysicalConstants.Intake.*;

import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreBeamBreak extends Command {
  private Intake intake;
  private Wrist wrist;

  private Timer timer;

  public ScoreBeamBreak() {
    intake = Intake.getInstance();
    wrist = Wrist.getInstance();
    
    timer = new Timer();

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    intake.setIntakeVelocity(AMP_SHOOT_SPEED);
  }

  @Override
  public void execute() {
    if (Intake.hasNote()) {
      intake.setIntakeVelocity(AMP_SHOOT_SPEED);
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeVelocity(0);
  }

  @Override
  public boolean isFinished() {
    return (!Intake.hasNote()) && timer.get() > 0.25;
  }
}
