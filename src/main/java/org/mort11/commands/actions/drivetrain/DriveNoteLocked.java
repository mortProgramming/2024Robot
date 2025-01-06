package org.mort11.commands.actions.drivetrain;

import static org.mort11.configuration.constants.PortConstants.Vision.NOTE_CAMERA;

import java.util.function.DoubleSupplier;

import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.LimelightHelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
			NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    		double tx = table.getEntry("tx").getDouble(0.0);

		drivetrain.setDrive(
			ChassisSpeeds.fromFieldRelativeSpeeds(
				translationXSupplier.getAsDouble(),
				translationYSupplier.getAsDouble(), 
				tx * -0.15,
                drivetrain.getGyroscopeRotation()
            )
        );

        // drivetrain.setAngleController(drivetrain.getGyroscopeRotation().getDegrees() + LimelightHelpers.getTX("limelight"));
		// drivetrain.setAngleController(tx * 0.005);

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
