// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Intake.INTAKE_SPEED;
import static frc.robot.utility.Constants.Wrist.WRIST_INTAKE_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeBeamBreak extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Intake intake = Intake.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private Timer timer = new Timer();
  private double endPosition = WRIST_REST_POSITION;
  public IntakeBeamBreak(double endPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake,wrist);
    this.endPosition = endPosition;
  }
  public IntakeBeamBreak() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake,wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intake.hasNote()) {
      intake.setIntakeVelocity(INTAKE_SPEED);
      wrist.setSetPoint(WRIST_INTAKE_POSITION);
    }else{
      timer.start();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    System.out.println(interrupted);
    intake.setIntakeVelocity(0);
      wrist.setSetPoint(endPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote() && timer.get() > .25;
  }
}
