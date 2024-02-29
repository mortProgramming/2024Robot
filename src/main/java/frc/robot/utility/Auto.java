package frc.robot.utility;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auton.OdometryCentered.Blue.ScoreAmpOB;
import frc.robot.commands.Auton.Timed.Taxi;
import frc.robot.commands.Auton.Timed.Blue.ScoreAmpB;
import frc.robot.commands.Auton.Timed.Blue.ScoreAmpTwiceB;
import frc.robot.commands.Auton.Timed.Red.ScoreAmpR;
import frc.robot.commands.Auton.Timed.Red.ScoreAmpTwiceR;

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
    	// autoChooser.addOption("Auton name", new autoncommand1, new autoncommand2);
		autoChooser.addOption("BlueOnePiece", new ScoreAmpB());
		autoChooser.addOption("BlueTwoPiece", new ScoreAmpTwiceB());
		autoChooser.addOption("RedOnePiece", new ScoreAmpR());
		autoChooser.addOption("RedTwoPiece", new ScoreAmpTwiceR());
		autoChooser.addOption("Taxi", new Taxi());
		autoChooser.addOption("PathPlanner TwoPiece", PathAuto.getTwoPiece());
		autoChooser.addOption("Odometer Thing", new ScoreAmpOB());
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