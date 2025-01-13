package org.mort11.commands.actions.drivetrain;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class Drive extends Command {
	private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;

	private boolean fieldOriented;

	public Drive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier, boolean fieldOriented) {
		drivetrain = Drivetrain.getInstance();

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		this.fieldOriented = fieldOriented;
		

		addRequirements(drivetrain);
	}

	public Drive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		this(translationXSupplier, translationYSupplier, rotationSupplier, false);
	}

    @Override
	public void execute() {
		if (fieldOriented) {
			drivetrain.setDrive(
				ChassisSpeeds.fromFieldRelativeSpeeds(
					translationXSupplier.getAsDouble(),
					translationYSupplier.getAsDouble(), 
					rotationSupplier.getAsDouble(),
					drivetrain.getIMURotation()));
		} else {
			drivetrain.setDrive(new ChassisSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(),
					rotationSupplier.getAsDouble()));
		}
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

