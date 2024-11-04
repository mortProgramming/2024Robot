package org.mort11.commands.actions.drivetrain;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveAtAngle extends Command {
    private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private double rotationSupplier;

	public DriveAtAngle(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			double rotationSupplier) {
		drivetrain = Drivetrain.getInstance();

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		

		addRequirements(drivetrain);
	}

    @Override
	public void execute() {
		drivetrain.setDrive(
			ChassisSpeeds.fromFieldRelativeSpeeds(
			translationXSupplier.getAsDouble(),
			translationYSupplier.getAsDouble(), 
			0,
			drivetrain.getGyroscopeRotation())
        );

        drivetrain.setAngleController(rotationSupplier);
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
