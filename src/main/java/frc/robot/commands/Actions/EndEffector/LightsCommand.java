package frc.robot.commands.Actions.EndEffector;

import static frc.robot.utility.Constants.Intake.AMP_SHOOT_SPEED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class LightsCommand extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Lights lights = Lights.getInstance();
  private Vision vision = Vision.getInstance();

  public LightsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Intake.getInstance().hasNote()) {
        lights.setLightsGreen();
        vision.setNoteCamLights(2);
    }
    else {
        lights.setLightsRed();
        vision.setNoteCamLights(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
