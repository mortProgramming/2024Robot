package org.mort11.commands.actions.endeffector;

import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Wrist;

import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeBeamBreak extends Command {
  private Intake intake;
  private Wrist wrist;
  private Timer timer;

  private double endPosition;

  public IntakeBeamBreak(double endPosition) {
    this.endPosition = endPosition;

    intake = Intake.getInstance();
    wrist = Wrist.getInstance();

    timer = new Timer();

    addRequirements(intake, wrist);
  }
  
  public IntakeBeamBreak() {
    this.endPosition = WRIST_REST_POS;

    intake = Intake.getInstance();
    wrist = Wrist.getInstance();

    timer = new Timer();

    addRequirements(intake, wrist);
  }

  @Override
  public void initialize() {
    wrist.setSetpoint(WRIST_INTAKE_POS);
    intake.setIntakeVelocity(INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if(Intake.hasNote()){
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    
    wrist.setSetpoint(endPosition);

    intake.setIntakeVelocity(0);
  }

  @Override
  public boolean isFinished() {
   return Intake.hasNote() && (timer.get() > SENSOR_MIN_TIME); 
   
  }
}
