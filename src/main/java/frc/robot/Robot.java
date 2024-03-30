// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import frc.robot.utility.Control;
import frc.robot.utility.Odometer;

import static frc.robot.utility.Constants.Wrist.WRIST_REST_POSITION;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private Alliance alliance = Alliance.Blue;
	private boolean regenSub = false;

	private RobotContainer robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		robotContainer = new RobotContainer();
		Odometer.OdometerInit();
		System.out.println("RobotInit");
	}

	/**
	 * This function is called every <b>20 ms</b>, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		
		
	}
	
	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		
		
	}

	@Override
	public void disabledPeriodic() {
		SmartDashboard.putBoolean("SUBSYSTEMS REGENERATED", regenSub);
		if(DriverStation.isDSAttached()){
			if(DriverStation.isFMSAttached()){
				if (DriverStation.getAlliance().get() != alliance){
					System.out.println("REGENERATING SUBSYSTEMS");
					Control.init();
					Control.configure();
					regenSub = true;
					alliance = DriverStation.getAlliance().get();
				}
			}
		}
		
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
		Wrist.getInstance().setVelocityMode(false);
		Wrist.getInstance().setSetPoint(WRIST_REST_POSITION);

	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		Odometer.updateOdometry();
		// Odometer.updateOdometryIgnoreLimelight();

	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		Wrist.getInstance().setServoPos(90);		
		Wrist.getInstance().setVelocityMode(false);
		Wrist.getInstance().setSetPoint(WRIST_REST_POSITION);
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		Odometer.updateOdometry();
	}

	@Override
	public void testInit() {
		
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
	}
}