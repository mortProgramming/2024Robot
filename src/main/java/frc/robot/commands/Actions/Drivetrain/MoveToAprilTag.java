package frc.robot.commands.Actions.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveToAprilTag extends Command{
    private Drivetrain drivetrain;
	private Pose3d pose;

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
		pose = LimelightHelpers.getBotPose3d_wpiBlue("");
	}

	@Override
	public void execute() {
		//Uses april tag controller to calculate X, Y, and Z positions
		double x = -drivetrain.getAprilTagXController().calculate(pose.getZ(), -1.4);
		double y =
				// LimelightHelpers.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagYController().calculate(pose.getX(), 0);
		// : 0;
		double omega =
				// LimelightHelpers.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagOmegaController().calculate(pose.getRotation().getY(), 0);
		// : 0;

		drivetrain.drive(new ChassisSpeeds(x, y, omega));
	}

	@Override
	public boolean isFinished() {
		//Checks to see if april tag controller is at its setpoint
		return !LimelightHelpers.getTV("")
				|| (drivetrain.getAprilTagXController().atSetpoint() && drivetrain.getAprilTagYController().atSetpoint()
						&& drivetrain.getAprilTagOmegaController().atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
	}
}
