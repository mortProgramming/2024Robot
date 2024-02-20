package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
	private static SendableChooser<Command> autoChooser;
	private static SendableChooser<Boolean> isBlue;

	/**
	 * Create autonomous commands and chooser
	 */
	public static void init() {
		autoChooser = new SendableChooser<Command>();
		isBlue = new SendableChooser<Boolean>();
		addAutoOptions();

		// put the auto chooser onto SmartDashboard
		SmartDashboard.putData(autoChooser);
		isBlue.setDefaultOption("Blue", true);
		isBlue.addOption("Red", false);

		SmartDashboard.putData("isBlue", isBlue);
	}

	/**
	 * 
	 */
	public static void addAutoOptions() {
		// By default, the nothing option is selected
		autoChooser.setDefaultOption("nothing", null);
    //autoChooser.addOption("Auton name", new autoncommand1, new autoncommand2);
	}

	/**
	 * 
	 * @return
	 */
	public static Boolean getIsBlue() {
		return isBlue.getSelected() != null ? isBlue.getSelected() : true;
	}

	/**
	 * @return selected auto from auto chooser
	 */
	public static Command getSelected() {
		return autoChooser.getSelected();
	}
}