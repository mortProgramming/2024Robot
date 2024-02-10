package frc.robot.util;

import static frc.robot.util.Constants.Intake.*;
import static frc.robot.util.Constants.PeripheralPorts.*;
import static frc.robot.util.Constants.RobotSpecs.*;
import static frc.robot.util.Constants.Wrist.*;
import static frc.robot.util.Constants.Climber.*;
import static frc.robot.util.Constants.Arm.*;
import frc.robot.util.Constants.Wrist.*;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.WristToVelocity;
import frc.robot.commands.Actions.EndEffector.WristToPosition;
import frc.robot.commands.Actions.EndEffector.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.ArmToVelocity;
import frc.robot.commands.Actions.EndEffector.ClimberToPosition;
import frc.robot.commands.Actions.EndEffector.ClimberToVelocity;
import frc.robot.commands.Actions.EndEffector.SetArmAndWristPos;
import frc.robot.commands.Teleop.DrivetrainCommand;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import edu.wpi.first.units.Units.*;


public class Control {
    private static CommandJoystick joystick;
	private static CommandJoystick throttle;
	private static CommandXboxController xboxController;

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Intake intake;
    private static Climber climber;
    private static Wrist wrist;
    Distance Meters = BaseUnits.Distance;
    //FINISHTHIS
    //private final MutableMeasure<Distance> mDistance = mutable(Meters.of(0));

    /**
     * FINISHTHIS
     */

    //  public static <U extends Unit<U>> MutableMeasure<U> mutable(Measure<U> measure){
    //     return new MutableMeasure<>(measure.magnitude(), measure.baseUnitMagnitude(), measure.unit());
    //  }


    public static void init() {
		joystick = new CommandJoystick(JOYSTICK);
		throttle = new CommandJoystick(THROTTLE);
		xboxController = new CommandXboxController(XBOX_CONTROLLER);

		drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();

        //FINISHTHIS

        // SysIdRoutine routine = new SysIdRoutine(
        //     new SysIdRoutine.Config(), 
        //     new SysIdRoutine.Mechanism(arm::setVoltage, 
            
        //     log -> {
        //         log.motor("drive-left").voltage(
        //             arm.getArmMaster().getMotorVoltage().mut_replace(
        //                 arm.getArmMaster().getMotorVoltage().getValueAsDouble() * RobotController.getBatteryVoltage(), Voltage))
        //                 .linearPosition(null).
        //             )
        //         );
        //         log.motor("drive-right").voltage(

        //         );
        //     }, 
            
        //     arm));
        
	}

    /**
     * 
     */
    public static void configure() {
		drivetrain.setDefaultCommand(
			new DrivetrainCommand(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist, true)
        );

                            //Competition controls
        // xboxController.rightBumper().toggleOnTrue(
        //     Commands.startEnd(() -> new IntakeToVelocity(INTAKE_SPEED), () -> new IntakeToVelocity(0)));
            
        // xboxController.rightTrigger().onTrue(new IntakeToVelocity(-INTAKE_SPEED));

        // xboxController.leftTrigger().onTrue(new WristToVelocity(WRIST_SPEED));

        // if(xboxController.getLeftTriggerAxis() > 0.2 && xboxController.getRightTriggerAxis() > 0.2){
        //     Commands.startEnd(() -> new ClimberToPosition(CLIMBER_MAX_POSITION), 
        //     () -> new ClimberToPosition(CLIMBER_REST_POSITION));
        // }

        // xboxController.y().toggleOnTrue(new SetArmAndWristPos(ARM_TRAP_POSITION, WRIST_TRAP_POSITION));


        xboxController.y().onTrue(new IntakeToVelocity(0.25));
        xboxController.y().onFalse(new IntakeToVelocity(0));
        xboxController.x().onTrue(new IntakeToVelocity(-1));
        xboxController.x().onFalse(new IntakeToVelocity(0));
        
        arm.setDefaultCommand(new ArmToVelocity(Control::getLeftJoystickY));
        
        wrist.setDefaultCommand(new WristToVelocity(Control::getRightJoystickY));
        
        xboxController.a().onTrue(new ClimberToVelocity(0.25));
        xboxController.a().onFalse(new ClimberToVelocity(0));
        xboxController.b().onTrue(new ClimberToVelocity(-0.25));
        xboxController.b().onFalse(new ClimberToVelocity(0));

        xboxController.povDown().onTrue(new ArmToPosition(ARM_REST_POSITION));
        xboxController.povDown().onFalse(new ArmToVelocity(Control::getLeftJoystickY));
        
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
    // public static double getJoystickX() {
	// 	return -(modifyAxis(joystick.getX(), throttle.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND) * 0.75;
	// }

    public static double getJoystickX(){
        return joystick.getX();
    }


    /**
     * 
     * @return
     */
	// public static double getJoystickY() {
	// 	return -(modifyAxis(joystick.getY(), throttle.getRawAxis(2)) * MAX_VELOCITY_METERS_PER_SECOND);
	// }

    public static double getJoystickY(){
        return joystick.getY();
    }

    /**
     * 
     * @return
     */
	// public static double getJoystickTwist() {
	// 	return -(modifyAxis(joystick.getTwist(), throttle.getRawAxis(2))
	// 			* MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
	// }

    public static double getJoystickTwist(){
        return -joystick.getTwist();
    }

    public static double getLeftJoystickY() {
        return xboxController.getLeftY() * 0.25;
    }

    public static double getRightJoystickY(){
        return xboxController.getRightY() * 0.25;
    }

    //FINISHTHIS
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    //     return routine.quasistatic(direction);
    // }
 
 }
