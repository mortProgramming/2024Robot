package org.mort11.commands.actions.drivetrain;

import org.mort11.configuration.Odometer;
import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToPos extends Command {
  private Drivetrain drivetrain;

  private double wantedX;
  private double wantedY;

  public MoveToPos(double wantedX, double wantedY) {
    drivetrain = Drivetrain.getInstance();

    this.wantedX = wantedX;
    this.wantedY = wantedY;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
   drivetrain.setPosController(Odometer.getPoseX(), Odometer.getPoseY(), wantedX, wantedY);
	}
  
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDrive(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return (drivetrain.getXControllerAtSetpoint() && 
      drivetrain.getYControllerAtSetpoint()
    );
  }
}
