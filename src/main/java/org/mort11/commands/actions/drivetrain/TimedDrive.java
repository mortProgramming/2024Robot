package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class TimedDrive extends Command{
  private Drivetrain drivetrain;

  private Timer timer;
  private double time;

  private double x;
  private double y;
  private double omega;
  /**
   * Moves the drivetrain a certain amount of time given movement parameters.
   * @param time
   * The amount of time to drive for
   * @param x
   * The velocity in the x direction
   * @param y
   * The velocity in the y direction
   * @param omega
   * The angular velocity
   */
  public TimedDrive(double time, double x, double y, double omega) {
    drivetrain = Drivetrain.getInstance();
    
    timer  = new Timer();
    this.time = time;

    this.x = x;
    this.y = y;
    this.omega = omega;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  @Override
  public void execute() {
    drivetrain.setDrive(
      new ChassisSpeeds(
      -y, x, omega
      )
    );
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
