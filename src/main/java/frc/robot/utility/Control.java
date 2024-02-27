package frc.robot.utility;

import static frc.robot.utility.Constants.PeripheralPorts.*;
import static frc.robot.utility.Constants.RobotSpecs.*;
import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Wrist.*;

import java.util.function.DoubleSupplier;

import static frc.robot.utility.Constants.Climber.*;
import static frc.robot.utility.Constants.Intake.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.WristToPosition;
import frc.robot.commands.Actions.EndEffector.WristToVelocity;
import frc.robot.commands.Actions.Drivetrain.MoveWithAngle;
import frc.robot.commands.Actions.EndEffector.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.ArmToVelocity;
import frc.robot.commands.Actions.EndEffector.ClimberToVelocity;
import frc.robot.commands.Teleop.DrivetrainCommand;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class Control {
    private static CommandJoystick joystick;
	private static CommandJoystick throttle;
	private static CommandXboxController xboxController;

    private static Joystick joystickN;
	private static Joystick throttleN;
	private static XboxController xboxControllerN;

    // private syta

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Intake intake;
    private static Climber climber;
    private static Wrist wrist;
    private static Odometer odometer;
    private static Vision vision;
    private static SysIdRoutine armRoutine;
    private static SysIdRoutine wristRoutine;
    
    private final static MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final static MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    private final static MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    
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

        // counter1 = 0;
        // counter1 = 0;

        

		drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();
        odometer = Odometer.getInstance();
        vision = Vision.getInstance();
        PathAuto.init();//Drivetrain methods must properly exist for the PathPlanner swerve configuration to work.


        armRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(.5).per(Seconds.of(1)),Volts.of(7),Seconds.of(10)),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> {arm.setVoltage(volts.in(Volts));},

            log -> {
                log.motor("armMotor")
                    .voltage(appliedVoltage.mut_replace(arm.getVoltage(), Volts))
                    .angularPosition(distance.mut_replace(arm.getArmMaster().getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(velocity.mut_replace(arm.getVelocity(), RotationsPerSecond));
            },
            arm));
        
        
    }

    /**
     * 
     */
    public static void configure() {
		drivetrain.setDefaultCommand(
			new DrivetrainCommand(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist, true)
        );

        joystick.trigger().whileTrue(new InstantCommand(() -> drivetrain.zeroGyroscope(180)));
        joystick.button(7).whileTrue(new InstantCommand(() -> odometer.resetOdometry(vision.getFieldPose())));

        // joystick.button(2).whileTrue(new InstantCommand(() -> drivetrain.setDefaultCommand(new MoveWithAngle(Control::getJoystickY, Control::getJoystickX, 270))));
        // joystick.button(2).whileFalse(new InstantCommand(() -> drivetrain.setDefaultCommand(new DrivetrainCommand(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist, true))));
        // joystick.button(2).whileTrue(new MoveWithAngle(Control::getJoystickY, Control::getJoystickX, 270));

//.withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        //Competition controls
        xboxController.rightBumper().onTrue(new IntakeToVelocity(INTAKE_SPEED));
        xboxController.rightBumper().onFalse(new IntakeToVelocity(0));

        xboxController.rightTrigger().onTrue(new IntakeToVelocity(AMP_SHOOT_SPEED));
        xboxController.rightTrigger().onFalse(new IntakeToVelocity(0));

        xboxController.a().onTrue(new IntakeToVelocity(SHOOTER_SHOOT_SPEED));
        xboxController.a().onFalse(new IntakeToVelocity(0));

        arm.setDefaultCommand(new ArmToVelocity(Control::getLeftJoystickY));

        xboxController.x().onTrue(new ArmToPosition(ARM_AMP_POSITION));
        xboxController.y().onTrue(new ArmToPosition(ARM_REST_POSITION));

        wrist.setDefaultCommand(new WristToVelocity(Control::getRightJoystickY));


        //arm and wrist switching with 
        xboxController.start().whileTrue(new InstantCommand(() -> arm.setVelocityMode(true)));
        xboxController.start().whileTrue(new InstantCommand(() -> wrist.setVelocityMode(true)));

        xboxController.start().whileTrue(new InstantCommand(() -> climber.setVelocityMode(false)));
        xboxController.start().whileFalse(new InstantCommand(() -> climber.setVelocityMode(true)));

        xboxController.povDown().toggleOnTrue(new InstantCommand(() -> climber.setRightSolenoid(1)));
        xboxController.povRight().toggleOnTrue(new InstantCommand(() -> climber.setRightSolenoid(0)));
        //xboxController.povDown().toggleOnFalse(new InstantCommand(() -> climber.setRightSolenoid(0)));
        xboxController.povDown().toggleOnTrue(new InstantCommand(() -> climber.setLeftSolenoid(1)));
        xboxController.povRight().toggleOnTrue(new InstantCommand(() -> climber.setLeftSolenoid(0)));
        //xboxController.povDown().toggleOnFalse(new InstantCommand(() -> climber.setLeftSolenoid(0)));


        climber.setDefaultCommand(new ClimberToVelocity(Control::getLeftJoystickY, Control::getRightJoystickY));


        // xboxController.a().onTrue(new ClimberToVelocity(() -> 0.25, true));
        // xboxController.a().onFalse(new ClimberToVelocity(() -> 0, true));
        // xboxController.b().onTrue(new ClimberToVelocity(() -> -0.25, true));
        // xboxController.b().onFalse(new ClimberToVelocity(0));

        // xboxController.povUp().toggleOnTrue(Commands.startEnd(
        //     () -> new WristToPosition(WRIST_REST_POSITION), () -> new WristToVelocity(Control::getRightJoystickY), 
        //     wrist));

        // if(xboxControllerN.getLeftBumperPressed() == true) {
        //     counter1++;
        // }

        xboxController.leftBumper().onTrue(new WristToPosition(WRIST_REST_POSITION));
        xboxController.leftTrigger().onTrue(new WristToPosition(WRIST_INTAKE_POSITION));

        // if(xboxController.getLeftTriggerAxis() > 0.2 && xboxController.getRightTriggerAxis() > 0.2){
        //     Commands.startEnd(() -> new ClimberToPosition(CLIMBER_MAX_POSITION), 
        //     () -> new ClimberToPosition(CLIMBER_REST_POSITION));
        // }



        // xboxController.y().toggleOnTrue(new SetArmAndWristPos(ARM_TRAP_POSITION, WRIST_TRAP_POSITION));

        // TESTING COMMANDS

        // xboxController.rightBumper().onTrue(new IntakeToVelocity(0.25));
        // xboxController.rightBumper().onFalse(new IntakeToVelocity(0));
        // xboxController.rightTrigger().onTrue(new IntakeToVelocity(-0.5));
        // xboxController.rightTrigger().onFalse(new IntakeToVelocity(0));

        // arm.setDefaultCommand(new ArmToVelocity(Control::getLeftJoystickY));
        // wrist.setDefaultCommand(new WristToVelocity(Control::getRightJoystickY));
        
        // xboxController.a().onTrue(new ClimberToVelocity(0.25));
        // xboxController.a().onFalse(new ClimberToVelocity(0));
        // xboxController.b().onTrue(new ClimberToVelocity(-0.25));
        // xboxController.b().onFalse(new ClimberToVelocity(0));

        // xboxController.x().whileTrue(new ArmToPosition(ARM_AMP_POSITION));
        // xboxController.y().whileTrue(new ArmToPosition(ARM_REST_POSITION));

        // // xboxController.povUp().toggleOnTrue(Commands.startEnd(
        // //     () -> new WristToPosition(WRIST_REST_POSITION), () -> new WristToVelocity(Control::getRightJoystickY), 
        // //     wrist));
        // xboxController.leftBumper().onTrue(new WristToPosition(WRIST_REST_POSITION));
        // xboxController.leftTrigger().onTrue(new WristToPosition(WRIST_INTAKE_POSITION));

        // if(xboxControllerN.getLeftBumperPressed() == true) {
        //     counter1++;
        // }


        

        

        // joystick.button(9).whileTrue(getQuasistaticDirectionalTest(Direction.kForward));
        // joystick.button(10).whileTrue(getQuasistaticDirectionalTest(Direction.kReverse));
        // joystick.button(11).whileTrue(getDynamicDirectionalTest(Direction.kForward));
        // joystick.button(12).whileTrue(getDynamicDirectionalTest(Direction.kReverse));
        
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
    public static double modifyAxis4(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value * value * value * value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
        // return value * throttleValue;
    }

    public static double modifyAxis5(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value * value * value * value * value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_THROTTLE) + MIN_THROTTLE);
        // return value * throttleValue;
    }

    public static double getThrottle() {
        return throttle.getRawAxis(2);
    }

    /**
     * 
     * @return
     */
    public static double getJoystickX() {
		return (modifyAxis1(joystick.getX(), throttle.getZ()) * MAX_VELOCITY_METERS_PER_SECOND) * 0.75;
	}

    // public static double getJoystickX(){
    //     return joystick.getX() * (throttle.getZ());
    // }


    /**
     * 
     * @return
     */
	public static double getJoystickY() {
		return (modifyAxis1(joystick.getY(), throttle.getZ()) * MAX_VELOCITY_METERS_PER_SECOND);
	}

    // public static double getJoystickY(){
    //     return joystick.getY() * (throttle.getZ());
    // }

    /**
     * 
     * @return
     */
	public static double getJoystickTwist() {
		return -0.3 * (modifyAxis1(joystick.getTwist(), throttle.getZ())
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
    public static Command getQuasistaticDirectionalTest(SysIdRoutine.Direction direction){
        return armRoutine.quasistatic(direction);
    }
    public static Command getDynamicDirectionalTest(SysIdRoutine.Direction direction){
        return armRoutine.dynamic(direction);
    }

    //FINISHTHIS
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    //     return routine.quasistatic(direction);
    // }
 
 }
