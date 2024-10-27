package org.mort11.commands.actions.endeffector;

import static org.mort11.configuration.constants.PortConstants.Vision.*;

import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Lights;
import org.mort11.subsystems.LimelightHelpers;
import org.mort11.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;

public class LightsCommand extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Lights lights = Lights.getInstance();

  public LightsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Intake.hasNote()) {
        lights.setLightsGreen();
        LimelightHelpers.setLEDMode_ForceBlink(NOTE_CAMERA);
        LimelightHelpers.setLEDMode_ForceBlink(TAG_CAMERA);
        
    }

    else {
        lights.setLightsBlue();
        LimelightHelpers.setLEDMode_ForceBlink(NOTE_CAMERA);
        LimelightHelpers.setLEDMode_ForceBlink(TAG_CAMERA);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.setLightsBlue();
    LimelightHelpers.setLEDMode_ForceOff(NOTE_CAMERA);
    LimelightHelpers.setLEDMode_ForceOff(TAG_CAMERA);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
