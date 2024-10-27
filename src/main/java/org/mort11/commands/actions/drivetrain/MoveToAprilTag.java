package org.mort11.commands.actions.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.LimelightHelpers;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveToAprilTag extends Command{
    private Drivetrain drivetrain;

    private int id;
	/**
	 * Moves the robot to an Apriltag on the field.
	 * @param id
	 * The Apriltag ID to move to
	 */
    public MoveToAprilTag(int id){
        drivetrain = Drivetrain.getInstance();

        this.id = id;
        addRequirements(drivetrain);

    }


	@Override
	public void initialize() {
		//Resets all April Tag control settings
		drivetrain.getAprilTagXController().reset();
		drivetrain.getAprilTagYController().reset();
		drivetrain.getAprilTagOmegaController().reset();
	}

	@Override
	public void execute() {
		//Uses april tag controller to calculate X, Y, and Z positions
		double x = -drivetrain.getAprilTagXController().calculate(LimelightHelpers.getTargetPose_RobotSpace(TAG_CAMERA)[2], -1.4);
		double y =
				// vision.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagYController().calculate(LimelightHelpers.getTargetPose_RobotSpace(TAG_CAMERA)[0], 0);
		// : 0;
		double omega =
				// vision.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagOmegaController().calculate(LimelightHelpers.getTargetPose_RobotSpace(TAG_CAMERA)[4], 0);
		// : 0;

		drivetrain.drive(new ChassisSpeeds(x, y, omega));
	}

	@Override
	public boolean isFinished() {
		//Checks to see if april tag controller is at its setpoint
		return !LimelightHelpers.getTV(TAG_CAMERA)
				|| (drivetrain.getAprilTagXController().atSetpoint() && drivetrain.getAprilTagYController().atSetpoint()
						&& drivetrain.getAprilTagOmegaController().atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
	}
}
