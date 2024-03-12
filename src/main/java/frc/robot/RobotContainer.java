package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utility.Auto;
import frc.robot.utility.Control;
import frc.robot.utility.PathAuto;

public class RobotContainer {
	private Drivetrain drivetrain;
	public RobotContainer() { 
    drivetrain = Drivetrain.getInstance();
		// initialize subsystems (TODO)
		while(DriverStation.isDSAttached()){
			//do nothing. Forces RobotContainer to wait until DS is connected

		}
		if(DriverStation.isFMSAttached()){
			System.out.println("DRIVERSTATION CONNECTED AND FMS CONNECTION ESTABLISHED, INITIALIZING SUBSYSTEMS");
		}else{
			System.out.println("NO FMS CONNECTION DETECTED, INITIALIZING SUBSYSTEMS");
		}
		//During a match, if these three lights are on, autonomous is good to run if on BLUE
		//if on red, final boolean should be false
		SmartDashboard.putBoolean("DS CONNECTED", DriverStation.isDSAttached());
		SmartDashboard.putBoolean("FMS CONNECTED", DriverStation.isFMSAttached());
		SmartDashboard.putBoolean("TRUE IF BLUE ALLIANCE", DriverStation.getAlliance().get() == Alliance.Blue);
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
