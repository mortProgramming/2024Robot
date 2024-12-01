package org.mort11.commands.actions.endeffector;

import static org.mort11.config.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.endeffector.pos.SetArmWristPos;
import org.mort11.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRest extends Command {
  private Intake intake;

  private Timer timer;

  public IntakeRest() {
    intake = Intake.getInstance();

    timer = new Timer();

    addRequirements(intake);
  }

  @Override
  public void execute() {
    if(intake.hasNote()) {
      timer.start();
    }

    SetArmWristPos.intake();
    intake.setIntakeVelocity(INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    
    SetArmWristPos.rest();

    intake.setIntakeVelocity(0);
  }

  @Override
  public boolean isFinished() {
   return intake.hasNote() && (timer.get() > SENSOR_MIN_TIME);
  }
}
