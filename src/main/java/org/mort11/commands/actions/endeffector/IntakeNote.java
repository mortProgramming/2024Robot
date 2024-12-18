package org.mort11.commands.actions.endeffector;

import static org.mort11.config.constants.PhysicalConstants.Intake.*;

import org.mort11.commands.actions.endeffector.pos.SetArmWristPos;
import org.mort11.subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {
  private Intake intake;

  private Timer timer;

  public IntakeNote() {
    intake = Intake.getInstance();

    timer = new Timer();

    addRequirements(intake);
  }

  @Override
  public void execute() {
    if(Intake.hasNote()) {
      timer.start();
    }

    SetArmWristPos.intake();
    intake.setIntakeVelocity(INTAKE_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();

    intake.setIntakeVelocity(0);
  }

  @Override
  public boolean isFinished() {
   return Intake.hasNote() && (timer.get() > SENSOR_MIN_TIME);
  }
}
