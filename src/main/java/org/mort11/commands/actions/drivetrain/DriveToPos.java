package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;

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
    drivetrain.setDrive(new ChassisSpeeds(
      drivetrain.getXController().calculate(drivetrain.getPoseX(), wantedX), 
      drivetrain.getYController().calculate(drivetrain.getPoseY(), wantedY), 
      drivetrain.getChassisSpeeds().omegaRadiansPerSecond
    ));
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
