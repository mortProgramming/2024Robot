package frc.robot;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auto;
import frc.robot.utility.Control;
import frc.robot.utility.Elastic;

public class RobotContainer {
	private Drivetrain drivetrain;
	public RobotContainer() { 
    	drivetrain = Drivetrain.getInstance();
		// initialize subsystems (TODO)
		
    	Control.init();
    	Control.configure();

		Auto.init();
		Elastic.Notification notification = new Elastic.Notification(
    Elastic.Notification.NotificationLevel.ERROR, "Error Notification", "This is an example error notification.");
		Elastic.sendNotification(notification);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return Auto.getSelected();
	}

}
