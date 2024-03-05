// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Intake.INTAKE_SPEED;
import static frc.robot.utility.Constants.Wrist.WRIST_INTAKE_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class IntakeBeamBreak extends Command {
  private Intake intake;
  private Wrist wrist;
  private Command wristDown = new WristToPosition(WRIST_INTAKE_POSITION);
  private Command wristUp = new WristToPosition(WRIST_REST_POSITION);
  private Command intakeStart = new WristToPosition(INTAKE_SPEED);
  private Command intakeStop = new IntakeToVelocity(0);


  /** Creates a new IntakeBeamBreak. */
  public IntakeBeamBreak() {
    intake = Intake.getInstance();
    wrist = Wrist.getInstance();
    addRequirements(intake,wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristUp.cancel();
    intakeStop.cancel();
    wristDown.schedule();
    intakeStart.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristDown.cancel();
    wristUp.cancel();
    wristUp.schedule();
    intakeStop.schedule();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
