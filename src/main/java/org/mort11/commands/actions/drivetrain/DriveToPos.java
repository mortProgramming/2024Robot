package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToPos extends Command {
  private Drivetrain drivetrain;

  private double wantedX;
  private double wantedY;

  public DriveToPos(double wantedX, double wantedY) {
    drivetrain = Drivetrain.getInstance();

    this.wantedX = wantedX;
    this.wantedY = wantedY;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
   drivetrain.getSwerveDrive().moveToPosition(new Pose2d(wantedX, wantedY, drivetrain.getIMURotation()));
	}
  
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (drivetrain.getXController().atSetpoint() && 
      drivetrain.getYController().atSetpoint()
    );
  }
}
