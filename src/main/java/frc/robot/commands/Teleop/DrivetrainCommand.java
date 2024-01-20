package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainCommand extends Command {
	private Drivetrain drivetrain;

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;

	private boolean fieldOriented;

	/**
	 * @param translationXSupplier
	 *            Supplier for x-axis movement
	 * @param translationYSupplier
	 *            Supplier for y-axis movement
	 * @param rotationSupplier
	 *            Supplier for rotational movement
	 * @param fieldOriented
	 *            Whether field-oriented drive is used
	 */
	public DrivetrainCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier, boolean fieldOriented) {
		drivetrain = Drivetrain.getInstance();

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		this.fieldOriented = fieldOriented;

		addRequirements(drivetrain);
	}

	/**
	 * Assumes robot-oriented drive
	 *
	 * @param translationXSupplier
	 *            Supplier for x-axis movement
	 * @param translationYSupplier
	 *            Supplier for y-axis movement
	 * @param rotationSupplier
	 *            Supplier for rotational movement
	 */
	public DrivetrainCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier) {
		this(translationXSupplier, translationYSupplier, rotationSupplier, false);
	}

    /**
     * 
     */
	@Override
	public void initialize() {

	}

    /**
     * Will propel the drivetrain to the speeds set by ChassisSpeeds
     */
	@Override
	public void execute() {
		if (fieldOriented) {
			drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(translationXSupplier.getAsDouble(),
					translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble(),
					drivetrain.getGyroscopeRotation()));
		} else {
			drivetrain.drive(new ChassisSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(),
					rotationSupplier.getAsDouble()));
		}

	}

    /**
     * 
     */
	@Override
	public boolean isFinished() {
		return false;
	}

    /**
     * 
     */
	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
	}
}

