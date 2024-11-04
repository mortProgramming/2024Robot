package org.mort11.configuration;

import org.mort11.commands.autons.odometered.ScoreAmpBlue;
import org.mort11.commands.autons.odometered.ScoreAmpRed;
import org.mort11.commands.autons.pathplanned.GetPlanned;
import org.mort11.commands.autons.timed.blue.TaxiB;
import org.mort11.commands.autons.timed.red.TaxiR;
import org.mort11.library.Swerve.PathPlanner;
import org.mort11.subsystems.Drivetrain;

import static org.mort11.configuration.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PIDConstants.Drivetrain.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

		AutoBuilder.configureHolonomic(
    		() -> Odometer.getOdometry().getEstimatedPosition(),
    		(Pose2d startPose) -> Odometer.resetOdometry(startPose), //reset odometry to a given pose. WILL ONLY RUN IF AUTON HAS A SET POSE, DOES NOTHING OTHERWISE. 
    		() -> drivetrain.getChassisSpeeds(),
    		(ChassisSpeeds robotRelativeOutput) -> drivetrain.setDrive(robotRelativeOutput),
    		new HolonomicPathFollowerConfig(
      			new PIDConstants(AUTON_POS_KP, AUTON_POS_KI, AUTON_POS_KD),
      			new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD),
      			AUTON_MAX_VELOCITY, //max Module Speed in M/s
      			DRIVEBASE_RADIUS_METERS,
       			new ReplanningConfig()), 
       		() -> !IO.isBlue(), //true when flips, default blue
    		drivetrain
		);

		// PathPlanner.configure(
		// 	drivetrain, drivetrain.swerveDrive, 
		// 	new PIDConstants(AUTON_POS_KP, AUTON_POS_KI, AUTON_POS_KD), 
		// 	new PIDConstants(AUTON_ROTATION_KP, AUTON_ROTATION_KI, AUTON_ROTATION_KD), 
		// 	DRIVEBASE_RADIUS_METERS
		// );
	}
	
	public static void addAutoOptions() {
		// By default, the nothing option is selected
		autoChooser.setDefaultOption("nothing", null);

		autoChooser.addOption("TaxiB", new TaxiB());
		autoChooser.addOption("TaxiR", new TaxiR());

		autoChooser.addOption("Odometer Thing Blue", new ScoreAmpBlue());
		autoChooser.addOption("Odometer Thing Red", new ScoreAmpRed());

		autoChooser.addOption("PathPlanner TwoPiece", GetPlanned.getTwoPiece());
		autoChooser.addOption("OneNote", GetPlanned.getChoreoOneNote());
		autoChooser.addOption("Gackley Auto", GetPlanned.getGackleyAuto());
		autoChooser.addOption("TwoPieceAmpSide", GetPlanned.getTwoPieceAmpSide());
		autoChooser.addOption("BieryTestAuto", GetPlanned.getBieryAuto());
	}

	public static Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}