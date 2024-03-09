// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Intake.SHOOTER_SHOOT_SPEED;
import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;
import static frc.robot.utility.Constants.Wrist.WRIST_SPIT_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Timer;

public class SpitNote extends Command {
  /** Creates a new SpitNote. */
  private Intake intake = Intake.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private Timer timer = new Timer();
  public SpitNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setSetPoint(WRIST_SPIT_POSITION);
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get()>.3){
      intake.setIntakeVelocity(SHOOTER_SHOOT_SPEED);
    }
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    System.out.println(interrupted);
    wrist.setSetPoint(WRIST_REST_POSITION);
    intake.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!intake.hasNote()) && timer.get()>1.5;
  }
}
