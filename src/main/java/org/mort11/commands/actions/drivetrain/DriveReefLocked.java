package org.mort11.commands.actions.drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Drivetrain;

public class DriveReefLocked extends Command {
    private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
    
    public DriveReefLocked(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
        drivetrain = Drivetrain.getInstance();

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		
		addRequirements(drivetrain);
    }

    @Override
	public void execute() {
		drivetrain.setDrive(
			ChassisSpeeds.fromFieldRelativeSpeeds(
			translationXSupplier.getAsDouble(),
			translationYSupplier.getAsDouble(), 
			0,
			drivetrain.getIMURotation())
        );

        drivetrain.setAngleController(0);
	}

    @Override
	public boolean isFinished() {
		return false;
	}

    @Override
	public void end(boolean interrupted) {
		drivetrain.setDrive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}
