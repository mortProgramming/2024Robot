package frc.robot.commands.Actions;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class TimedDrive extends Command{
  private Drivetrain drivetrain;
  private Timer timer;
  private double time;

  private double x;
  private double y;
  private double omega;

  public TimedDrive(double time, double x, double y, double omega) {
    drivetrain = Drivetrain.getInstance();
    
    timer  = new Timer();
    this.time = time;

    this.x = x;
    this.y = y;
    this.omega = omega;
    
    addRequirements(drivetrain);
  }

  /**
   * Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(x, y, omega));
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
    return timer.get() > time;
  }
}
