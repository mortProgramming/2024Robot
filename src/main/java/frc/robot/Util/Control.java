package frc.robot.util;

import static frc.robot.util.Constants.Intake.INTAKE_SPEED;
import static frc.robot.util.Constants.PeripheralPorts.*;
import static frc.robot.util.Constants.RobotSpecs.*;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Teleop.DrivetrainCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Constants.Climber;
import frc.robot.util.Constants.Intake;
import frc.robot.subsystems.Arm;



public class Control {
    private static CommandJoystick joystick;
	private static CommandJoystick throttle;
	private static CommandXboxController xboxController;

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Intake intake;
    private static Climber climber;

    /**
     * 
     */
    public static void init() {
		joystick = new CommandJoystick(JOYSTICK);
		throttle = new CommandJoystick(THROTTLE);
		xboxController = new CommandXboxController(XBOX_CONTROLLER);

		drivetrain = Drivetrain.getInstance();
        
	}

    /**
     * 
     */
    public static void configure() {
		drivetrain.setDefaultCommand(
			new DrivetrainCommand(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist, true)
        );


        xboxController.rightBumper().toggleOnTrue(new IntakeToVelocity(INTAKE_SPEED));
        xboxController.rightTrigger().onTrue(new IntakeToVelocity(-INTAKE_SPEED));
        
        

    }

    //where the axis doesn't take in values near 0, and starts past 0

    /**
     * scaled deadband - removing a value of the region around the zero value and scaling the rest to fit
     * 
     * @param value the total region that has a region near 0 that is a margin of error that needs to be corrected
     * @param deadband the margin of error around a zero point which is considered to be zero. 
     * @return the total region that has had the margin of error 0ed
     */
    public static double deadband(double value, double deadband) {

        if (Math.abs(value) > deadband) {
            //deadband is the region defined as +- deviation equal to 0

            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        }

        else {
            return 0.0;
        }
    }

    //raw data of value setting anything less than deadband equal to 0

    /**
     * 
     * @param value
     * @param deadband
     * @return
     */
    public static double unScaledDeadband(double value, double deadband) {

        if (Math.abs(value) > deadband) {
            return (value);
        }

        else {
            return 0.0;
        }
    }

    //deadband with a custom max and min value
    /**
     * 
     * @param value
     * @param throttleValue
     * @return
     */
    public static double modifyAxis(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        throttleValue = (throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
    }

    /**
     * 
     * @return
     */
    public static double getJoystickX() {
		return -(modifyAxis(joystick.getX(), throttle.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND) * 0.75;
	}


    /**
     * 
     * @return
     */
	public static double getJoystickY() {
		return -(modifyAxis(joystick.getY(), throttle.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND);
	}

    /**
     * 
     * @return
     */
	public static double getJoystickTwist() {
		return -(modifyAxis(joystick.getTwist(), throttle.getRawAxis(2))
				* MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
	}
}
