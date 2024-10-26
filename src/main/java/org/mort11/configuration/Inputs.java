package org.mort11.configuration;

import static org.mort11.configuration.Constants.RobotSpecs.*;
import static org.mort11.configuration.Constants.PeripheralPorts.*;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Inputs {

    public static CommandJoystick joystick;
	public static CommandXboxController xboxController;

    public static void init() {
		joystick = new CommandJoystick(JOYSTICK);
        xboxController = new CommandXboxController(CONTROLLER);

        // joystick.setXChannel(JOYSTICK_X_CHANNEL);
        // joystick.setYChannel(JOYSTICK_Y_CHANNEL);
        // joystick.setTwistChannel(JOYSTICK_TWIST_CHANNEL);
        // joystick.setThrottleChannel(THROTTLE_CHANNEL);

        // throttle.setThrottleChannel(THROTTLE_CHANNEL);
    }
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

     public static double modifyAxis1(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
    }

    public static double modifyAxis2(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value * value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
        // return value * throttleValue;
    }

    public static double modifyAxisTwist(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_ROTATE_THROTTLE - MIN_ROTATE_THROTTLE) + MIN_ROTATE_THROTTLE);
        // return value * throttleValue;
    }

    public static double modifyAxis5(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value * value * value * value * value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
        // return value * throttleValue;
    }

    /**
     * 
     * @return
     */
    public static double getJoystickX() {
		return -(modifyAxis1(joystick.getX(), joystick.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND) * 0.75;
	}

    /**
     * 
     * @return
     */
	public static double getJoystickY() {
		return -(modifyAxis1(joystick.getY(), joystick.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND);
	}

    /**
     * 
     * @return
     */
	public static double getJoystickTwist() {
		return -0.3 * (modifyAxisTwist(joystick.getRawAxis(3), joystick.getRawAxis(2))
				* MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
	}

    // public static double getJoystickTwist(){
    //     return (joystick.getTwist() * (throttle.getZ() + 1) / 2) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    // }

    public static double getLeftJoystickY() {
        return xboxController.getLeftY() * -0.25;
    }

    public static double getRightJoystickY(){
        return xboxController.getRightY() * 0.25;
    }
}
