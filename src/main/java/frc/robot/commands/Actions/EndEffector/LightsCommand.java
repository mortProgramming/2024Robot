package frc.robot.commands.Actions.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.*;

public class LightsCommand extends Command {
  /** Creates a new IntakeBeamBreak. */
  private Lights lights = Lights.getInstance();
  private Vision vision = Vision.getInstance();
  private Arm arm = Arm.getInstance();

  public LightsCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  //   if (Intake.hasNote()) {
  //       lights.setLightsGreen();
  //       vision.setCamLights(2);
        
  //   }

  //   else {
  //       lights.setLightsBlue();
  //       vision.setCamLights(1);
  //   }
  // }
  @Override
  public void execute(){
    if (arm.ArmPositionColor() == true){
      lights.setLightsOrange();
    }
    }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    vision.setCamLights(1);
    lights.setLightsBlue();
  }
//lights method
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
