package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auton.OdometryCentered.Blue.ScoreAmpOB;
import frc.robot.commands.Auton.OdometryCentered.Red.ScoreAmpOR;
import frc.robot.commands.Auton.Timed.Blue.ScoreAmpB;
import frc.robot.commands.Auton.Timed.Blue.ScoreAmpTaxiB;
import frc.robot.commands.Auton.Timed.Blue.ScoreAmpTwiceB;
import frc.robot.commands.Auton.Timed.Blue.TaxiB;
import frc.robot.commands.Auton.Timed.Red.ScoreAmpR;
import frc.robot.commands.Auton.Timed.Red.ScoreAmpTaxiR;
import frc.robot.commands.Auton.Timed.Red.ScoreAmpTwiceR;
import frc.robot.commands.Auton.Timed.Red.TaxiR;

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
		// if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) {
		// 	autoChooser.addOption("BlueOnePiece", new ScoreAmpB());
		// 	autoChooser.addOption("BlueTwoPiece", new ScoreAmpTwiceB());
		// }

		// else if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
		// 	autoChooser.addOption("RedOnePiece", new ScoreAmpR());
		// 	autoChooser.addOption("RedTwoPiece", new ScoreAmpTwiceR());
		// }

		// else {
		// 	autoChooser.addOption("BlueOnePiece", new ScoreAmpB());
		// 	autoChooser.addOption("BlueTwoPiece", new ScoreAmpTwiceB());
		// 	autoChooser.addOption("RedOnePiece", new ScoreAmpR());
		// 	autoChooser.addOption("RedTwoPiece", new ScoreAmpTwiceR());
		// }
		//team irrelevant autons
		autoChooser.addOption("TaxiB", new TaxiB());
		autoChooser.addOption("TaxiR", new TaxiR());

		autoChooser.addOption("PathPlanner TwoPiece", PathAuto.getTwoPiece());
		autoChooser.addOption("Odometer Thing Blue", new ScoreAmpOB());
		autoChooser.addOption("Odometer Thing Red", new ScoreAmpOR());
		autoChooser.addOption("OneNote", PathAuto.getChoreoOneNote());
		autoChooser.addOption("Gackley Auto", PathAuto.getGackleyAuto());
		autoChooser.addOption("TwoPieceAmpSide", PathAuto.getTwoPieceAmpSide());

		//things
		// autoChooser.addOption("BlueOnePiece", new ScoreAmpB());
		// autoChooser.addOption("BlueTwoPiece", new ScoreAmpTwiceB());
		// autoChooser.addOption("RedOnePiece", new ScoreAmpR());
		// autoChooser.addOption("RedTwoPiece", new ScoreAmpTwiceR());
		// autoChooser.addOption("BlueOneAndTaxi", new ScoreAmpTaxiB());
		// autoChooser.addOption("RedOneAndTaxi", new ScoreAmpTaxiR());
		autoChooser.addOption("BieryTestAuto", PathAuto.getBieryAuto());
	}

	/**
	 * 
	 * @return
	 */
	public static Boolean getIsBlue() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}

	/**
	 * @return selected auto from auto chooser
	 */
	public static Command getSelected() {
		return autoChooser.getSelected();
	}
}