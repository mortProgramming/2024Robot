package org.mort11.commands.actions.drivetrain;

import static org.mort11.config.constants.PortConstants.Vision.*;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Drivetrain;
import com.LimelightHelpers.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveNoteLocked extends Command {
    private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;

	public DriveNoteLocked(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier) {
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
                Rotation2d.fromDegrees(0)
            )
        );

        drivetrain.setAngleController(drivetrain.getIMURotation().getDegrees() + LimelightHelpers.getTX(NOTE_CAMERA));
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
	