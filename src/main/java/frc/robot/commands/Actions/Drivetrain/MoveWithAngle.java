package frc.robot.commands.Actions.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class MoveWithAngle extends Command{
  private Drivetrain drivetrain;
  private double x;
  private double y;
  private double angle;
  private boolean fieldOriented;
  /**
   * Moves the drivetrain a certain amount of time given movement parameters.
   * @param time
   * The amount of time to drive for
   * @param x
   * The velocity in the x direction
   * @param y
   * The velocity in the y direction
   * @param angle
   * The angular velocity
   */
  public MoveWithAngle(double x, double y, double angle) {
    drivetrain = Drivetrain.getInstance();

    this.x = x;
    this.y = y;
    this.angle = angle;
    this.fieldOriented = true;
    addRequirements(drivetrain);
  }

  public MoveWithAngle(DoubleSupplier x, DoubleSupplier y, double angle) {
    drivetrain = Drivetrain.getInstance();

    this.x = x.getAsDouble();
    this.y = y.getAsDouble();
    this.angle = angle;
    this.fieldOriented = true;
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
    // if (fieldOriented) {
	// 		drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
    //     x, y, omega,
	// 				drivetrain.getGyroscopeRotation()));
	// 	} else {
	// 		drivetrain.drive(new ChassisSpeeds(
    //     x, y, omega));
	// 	}
	drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        x, y, drivetrain.getRotateToAngleController().calculate(drivetrain.getGyroscopeRotation().getDegrees() + 180, angle),
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
    return false;
  }
}