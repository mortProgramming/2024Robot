package org.mort11.config;

import static org.mort11.config.constants.PIDConstants.Drivetrain.*;
import static org.mort11.config.constants.PhysicalConstants.Drivetrain.*;

import org.mort11.commands.autons.odometered.ScoreAmpBlue;
import org.mort11.commands.autons.odometered.ScoreAmpRed;
import org.mort11.commands.autons.pathplanned.BasicCommands;
import org.mort11.commands.autons.timed.blue.TaxiB;
import org.mort11.commands.autons.timed.red.TaxiR;
import org.mort11.mortlib.swerve.PathPlanner;
import org.mort11.subsystems.Drivetrain;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Auto {
	private static Drivetrain drivetrain;

	private static SendableChooser<Command> autoChooser;
	
	public static void configure() {
		drivetrain = Drivetrain.getInstance();

		autoChooser = new SendableChooser<Command>();
		configureAutoBuilder();
		addAutoOptions();

		SmartDashboard.putData(autoChooser);
	}

	public static void configureAutoBuilder() {
		drivetrain.setGyroscopeZero(0);

		PathPlanner.configure(
			drivetrain, drivetrain.getSwerveDrive(),
			() -> Odometer.getOdometry().getEstimatedPosition(), (Pose2d startPose) -> Odometer.resetOdometry(startPose),
			new PIDConstants(AUTON_POS_KP, AUTON_POS_KI, AUTON_POS_KD), 
			new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD), 
			DRIVEBASE_RADIUS_METERS
		);
	}
	
	public static void addAutoOptions() {
		// By default, the nothing option is selected
		autoChooser.setDefaultOption("nothing", null);

		autoChooser.addOption("TaxiB", new TaxiB());
		autoChooser.addOption("TaxiR", new TaxiR());

		autoChooser.addOption("Odometer Thing Blue", new ScoreAmpBlue());
		autoChooser.addOption("Odometer Thing Red", new ScoreAmpRed());

		autoChooser.addOption("PathPlanner TwoPiece", getPlanned("PathPlanned2Piece"));
		autoChooser.addOption("OneNote", getPlanned("OneNote"));
		autoChooser.addOption("Gackley Auto", getPlanned("GackleyAuto1"));
		autoChooser.addOption("TwoPieceAmpSide", getPlanned("TwoPieceAmpSide"));
		autoChooser.addOption("BieryTestAuto", getPlanned("BieryWildAuto"));
	}

	public static Command getPlanned(String plan) {
		BasicCommands.setCommands();

		return new PathPlannerAuto(plan);
	}

	public static Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
