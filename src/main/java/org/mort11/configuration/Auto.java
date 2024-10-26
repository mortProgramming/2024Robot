package org.mort11.configuration;

import org.mort11.commands.Auton.OdometryCentered.Blue.ScoreAmpOB;
import org.mort11.commands.Auton.OdometryCentered.Red.ScoreAmpOR;
import org.mort11.commands.Auton.Timed.Blue.TaxiB;
import org.mort11.commands.Auton.Timed.Red.TaxiR;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
	private static SendableChooser<Command> autoChooser;
	
	/**
	 * Create autonomous commands and chooser
	 */
	public static void configure() {
		autoChooser = new SendableChooser<Command>();
		addAutoOptions();

		SmartDashboard.putData(autoChooser);
	}
	
	public static void addAutoOptions() {
		// By default, the nothing option is selected
		autoChooser.setDefaultOption("nothing", null);

		autoChooser.addOption("TaxiB", new TaxiB());
		autoChooser.addOption("TaxiR", new TaxiR());

		autoChooser.addOption("PathPlanner TwoPiece", PathAuto.getTwoPiece());
		autoChooser.addOption("Odometer Thing Blue", new ScoreAmpOB());
		autoChooser.addOption("Odometer Thing Red", new ScoreAmpOR());
		autoChooser.addOption("OneNote", PathAuto.getChoreoOneNote());
		autoChooser.addOption("Gackley Auto", PathAuto.getGackleyAuto());
		autoChooser.addOption("TwoPieceAmpSide", PathAuto.getTwoPieceAmpSide());
		autoChooser.addOption("BieryTestAuto", PathAuto.getBieryAuto());
	}

	/**
	 * @return selected auto from auto chooser
	 */
	public static Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}