package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import javax.swing.GroupLayout.Alignment;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DrivetrainCommand extends Command {
	private Drivetrain drivetrain = Drivetrain.getInstance();

	private DoubleSupplier translationXSupplier;
	private DoubleSupplier translationYSupplier;
	private DoubleSupplier rotationSupplier;
	private Trigger ampLockButton;
	private Trigger noteLockButton;



	private boolean fieldOriented;
	private ChassisSpeeds chassisSpeeds;

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


		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;

		this.fieldOriented = fieldOriented;
		

		addRequirements(drivetrain);
	}
	public DrivetrainCommand(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
			DoubleSupplier rotationSupplier, boolean fieldOriented, Trigger ampLockButton, Trigger noteLockButton) {


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

		//Money Mikes one command driver control
		// if (fieldOriented){
		// 	if(noteLockButton.getAsBoolean()){
		// 		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		// 		translationXSupplier.getAsDouble(), 
		// 		translationYSupplier.getAsDouble(), 
		// 		drivetrain.getRotateToAngleController().calculate(vision.getNoteXDegrees(),0), 
		// 		drivetrain.getGyroscopeRotation());
		// 	}else if(ampLockButton.getAsBoolean()) {
		// 		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		// 		translationXSupplier.getAsDouble(),
		// 		translationYSupplier.getAsDouble(),
		// 		drivetrain.getRotateToAngleController().calculate(Drivetrain.toCircle(drivetrain.getGyroscopeRotation().getDegrees()), DriverStation.getAlliance().get() == Alliance.Blue ? 90 : 270), 
		// 		drivetrain.getGyroscopeRotation());
		// 	}else{
		// 		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
		// 		translationXSupplier.getAsDouble(),
		// 		translationYSupplier.getAsDouble(),
		// 		rotationSupplier.getAsDouble(),
		// 		drivetrain.getGyroscopeRotation());
		// 	}
		
		// }else{
		// 	chassisSpeeds = new ChassisSpeeds(translationXSupplier.getAsDouble(), translationYSupplier.getAsDouble(), rotationSupplier.getAsDouble());
		// }

		// drivetrain.drive(chassisSpeeds);

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

