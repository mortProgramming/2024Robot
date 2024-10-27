// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.mort11.commands.actions.endeffector;

import static org.mort11.configuration.constants.PhysicalConstants.Intake.AMP_SHOOT_SPEED;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.INTAKE_SPEED;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.WRIST_INTAKE_POS;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.WRIST_REST_POS;

import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreBeamBreak extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Intake intake = Intake.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private Timer timer = new Timer();

  public ScoreBeamBreak() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake,wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeVelocity(AMP_SHOOT_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.hasNote()) {
      intake.setIntakeVelocity(AMP_SHOOT_SPEED);
      timer.start();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println(interrupted);
    intake.setIntakeVelocity(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!intake.hasNote()) && timer.get() > 0.25;
  }
}
