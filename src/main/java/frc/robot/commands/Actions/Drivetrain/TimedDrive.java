package frc.robot.commands.Actions.Drivetrain;

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
  private boolean fieldOriented;
  private boolean isAngleKept;
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
    this.fieldOriented = true;
    this.isAngleKept = false;
    addRequirements(drivetrain);
  }
  public TimedDrive(double time, double x, double y, double omega, boolean fieldOriented) {
    drivetrain = Drivetrain.getInstance();
    
    timer  = new Timer();
    this.time = time;

    this.x = x;
    this.y = y;
    this.omega = omega;
    this.fieldOriented = fieldOriented;
    this.isAngleKept = false;
    addRequirements(drivetrain);
  }

  public TimedDrive(boolean isAngleKept, double time, double x, double y, double omega) {
    drivetrain = Drivetrain.getInstance();
    
    timer  = new Timer();
    this.time = time;

    this.x = x;
    this.y = y;
    this.omega = omega;
    this.fieldOriented = false;
    this.isAngleKept = isAngleKept;

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
    drivetrain.setIsAngleKept(isAngleKept);
    if (isAngleKept) {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        -y, x, 0,
        drivetrain.getGyroscopeRotation()
        ));
        drivetrain.setKeptAngle(omega);
    }
    else if (fieldOriented) {
			drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        -y, x, omega,
					drivetrain.getGyroscopeRotation()));
		} else {
			  drivetrain.drive(new ChassisSpeeds(
        -y, x, omega));
		}
    
    
    // if (fieldOriented) {
		// 	drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
    //     -y, x, omega,
		// 			drivetrain.getGyroscopeRotation()));
		// } else {
		// 	drivetrain.drive(new ChassisSpeeds(
    //     -y, x, omega));
		// }
	}
  /**
   * Called once the command ends or is interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setIsAngleKept(false);
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
