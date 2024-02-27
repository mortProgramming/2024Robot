package frc.robot.commands.Actions.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Odometer;

public class MoveToPosition extends Command {
    private Drivetrain drivetrain;
  private double wantedX;
  private double wantedY;
  private double wantedAngle;

  private Odometer odometer;
  /*
   * @param wantedX
   * The velocity in the x direction
   * @param wantedY
   * The velocity in the y direction
   * @param wantedAngle
   * The angular velocity
   */
  public MoveToPosition(double wantedX, double wantedY, double wantedAngle) {
    drivetrain = Drivetrain.getInstance();
    odometer = Odometer.getInstance();

    this.wantedX = wantedX;
    this.wantedY = wantedY;
    this.wantedAngle = wantedAngle;

    addRequirements(drivetrain);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
	drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        drivetrain.getXToPositiController().calculate(odometer.getPoseX(), wantedX), 
        drivetrain.getYToPositiController().calculate(odometer.getPoseY(), wantedY), 
        drivetrain.getRotateToAngleController().calculate(drivetrain.getGyroscopeRotation().getDegrees() + 180, wantedAngle),
		drivetrain.getGyroscopeRotation()));
	}
  
  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0, 0, 0));
  }

  /**
   * Returns true when the command should end.
   */
  @Override
  public boolean isFinished() {
    return (drivetrain.getXToPositiController().atSetpoint() && 
    drivetrain.getYToPositiController().atSetpoint());
  }
}
