package org.mort11.commands.actions.endeffector;

import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.Command;

public class Lighting extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Lights lights;

  public Lighting() {
    lights = Lights.getInstance();

    addRequirements(lights);
  }

  @Override
  public void execute() {
    if (Intake.hasNote()) {
      lights.setLightsGreen();

      Lights.setLimelightsBlink();
        
    }

    else {
      lights.setLightsBlue();

      Lights.setLimelightsOff();
    }
  }

  @Override
  public void end(boolean interrupted) {
    lights.setLightsBlue();

    Lights.setLimelightsOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
