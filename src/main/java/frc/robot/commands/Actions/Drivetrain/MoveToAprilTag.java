package frc.robot.commands.Actions.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utility.Constants.Vision.Pipeline;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveToAprilTag extends Command{
    private Drivetrain drivetrain;
    private Vision vision;

    private int id;
	/**
	 * Moves the robot to an Apriltag on the field.
	 * @param id
	 * The Apriltag ID to move to
	 */
    public MoveToAprilTag(int id){
        drivetrain = Drivetrain.getInstance();
        vision = Vision.getInstance();

        this.id = id;
        addRequirements(drivetrain, vision);

    }


	@Override
	public void initialize() {
		vision.setPipeline(Pipeline.APRIL_TAG, id);
		drivetrain.getAprilTagXController().reset();
		drivetrain.getAprilTagYController().reset();
		drivetrain.getAprilTagOmegaController().reset();
	}

	@Override
	public void execute() {
		double x = -drivetrain.getAprilTagXController().calculate(vision.getCamTranslationZ(), -1.4);
		double y =
				// vision.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagYController().calculate(vision.getCamTranslationX(), 0);
		// : 0;
		double omega =
				// vision.getCamTranZ() > -1.5 ?
				-drivetrain.getAprilTagOmegaController().calculate(vision.getCamTranslationYaw(), 0);
		// : 0;

		drivetrain.drive(new ChassisSpeeds(x, y, omega));
	}

	@Override
	public boolean isFinished() {
		return !vision.hasTarget()
				|| (drivetrain.getAprilTagXController().atSetpoint() && drivetrain.getAprilTagYController().atSetpoint()
						&& drivetrain.getAprilTagOmegaController().atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.drive(new ChassisSpeeds(0, 0, 0));
	}
}
