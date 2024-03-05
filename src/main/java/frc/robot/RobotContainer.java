package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.Auto;
import frc.robot.utility.Control;


public class RobotContainer {

	public RobotContainer() { 

		// initialize subsystems (TODO)
    	Control.init();
    	Control.configure();

		Auto.init();
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
