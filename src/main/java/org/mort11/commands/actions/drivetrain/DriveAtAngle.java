package org.mort11.commands.actions.drivetrain;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveAtAngle extends Command {
    private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private double rotationInDegreesSupplier;

	public DriveAtAngle(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			double rotationInDegreesSupplier) {
		drivetrain = Drivetrain.getInstance();

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationInDegreesSupplier = rotationInDegreesSupplier;
		

		addRequirements(drivetrain);
	}

    @Override
	public void execute() {
		drivetrain.setDrive(
			new ChassisSpeeds(
				translationXSupplier.getAsDouble(),
				translationYSupplier.getAsDouble(), 
				drivetrain.calculateRotateController(rotationInDegreesSupplier)
			)
        );
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
