package frc.robot.utility;

import static frc.robot.utility.Constants.PeripheralPorts.*;
import static frc.robot.utility.Constants.RobotSpecs.*;
import static frc.robot.utility.Constants.Arm.*;
import static frc.robot.utility.Constants.Wrist.*;

import java.util.function.DoubleSupplier;

import static frc.robot.utility.Constants.Climber.*;
import static frc.robot.utility.Constants.Intake.*;
import static frc.robot.utility.Constants.Lights.*;


import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Actions.EndEffector.IntakeToVelocity;
import frc.robot.commands.Actions.EndEffector.LightsCommand;
import frc.robot.commands.Actions.EndEffector.ArmWrist.ArmToPosition;
import frc.robot.commands.Actions.EndEffector.ArmWrist.ArmToVelocity;
import frc.robot.commands.Actions.EndEffector.ArmWrist.SetArmAndWristPos;
import frc.robot.commands.Actions.EndEffector.ArmWrist.WristToPosition;
import frc.robot.commands.Actions.EndEffector.ArmWrist.WristToVelocity;
import frc.robot.commands.Actions.Drivetrain.MoveToAprilTag;
import frc.robot.commands.Actions.EndEffector.ClimberToPosition;
import frc.robot.commands.Actions.EndEffector.ClimberToVelocity;
import frc.robot.commands.Actions.EndEffector.IntakeBeamBreak;
import frc.robot.commands.Teleop.DrivetrainCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.MutableMeasure.mutable;

public class Control {
    private Alliance defaultAlliance = Alliance.Blue;
    private static CommandJoystick joystick;
	private static CommandJoystick throttle;
	private static CommandXboxController xboxController;
    private static DoubleSupplier zeroSupplier = new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return 0.0;
        }
    };;

    // private syta

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Climber climber;
    private static Wrist wrist;
    private static Vision vision;
    private static Lights lights;

    private static SysIdRoutine armRoutine;
    private static SysIdRoutine wristRoutine;
    private static Intake intake;
    
    private final static MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final static MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    private final static MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    
    


    public static void init() {
        
        
		joystick = new CommandJoystick(JOYSTICK);
		throttle = new CommandJoystick(THROTTLE);
		xboxController = new CommandXboxController(XBOX_CONTROLLER); 
        System.out.println("Peripheral init");


		drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();
        vision = Vision.getInstance();
        lights = Lights.getInstance();
        System.out.println("Subsystem init");

        PathAuto.init();//Drivetrain methods must properly exist for the PathPlanner swerve configuration to work.
        System.out.println("auto init");;

    
        
    }

    public static void configure() {
		drivetrain.setDefaultCommand(
			new DrivetrainCommand(Control::getJoystickY, Control::getJoystickX, Control::getJoystickTwist, true)
        );
        arm.setDefaultCommand(new ArmToVelocity(Control::getLeftJoystickY));
       // wrist.setDefaultCommand(new WristToVelocity(Control::getRightJoystickY));

        joystick.trigger().whileTrue(new InstantCommand(() -> drivetrain.zeroGyroscope(0)));
        joystick.button(7).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(vision.getFieldPose())));

        joystick.button(9).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(3).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));


        joystick.button(9).or(joystick.button(3)).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(9).whileTrue(new InstantCommand(() -> drivetrain.setKeptAngleRelative(90)));
        joystick.button(3).whileTrue(
            new InstantCommand(() -> drivetrain.setKeptAngle(
                Drivetrain.toCircle(
                    180 + drivetrain.getGyroscopeRotation().getDegrees() + vision.getNoteXDegrees())
                )
            )
        );

        joystick.button(9).onFalse(new InstantCommand(() -> drivetrain.setIsAngleKept(false)));
        joystick.button(3).onFalse(new InstantCommand(() -> drivetrain.setIsAngleKept(false)));


        joystick.button(8).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(new Pose2d(1.515,7.395, Rotation2d.fromRadians(-1.571)))));

        joystick.button(6).whileTrue(new MoveToAprilTag(15));

      //  joystick.button(5).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(0.4, 7.5, 90)));


        xboxController.rightBumper().whileTrue(new IntakeBeamBreak(WRIST_REST_POSITION));

        xboxController.rightTrigger().onTrue(new IntakeToVelocity(AMP_SHOOT_SPEED));
        xboxController.rightTrigger().onFalse(new IntakeToVelocity(0));

        xboxController.a().onTrue(new IntakeToVelocity(SHOOTER_SHOOT_SPEED));
        xboxController.a().onFalse(new IntakeToVelocity(0));

        

        xboxController.x().onTrue(new ArmToPosition(ARM_AMP_POSITION));
        xboxController.y().onTrue(new ArmToPosition(ARM_REST_POSITION));
        xboxController.b().onTrue(new ArmToPosition(ARM_TRAP_POSITION));
        xboxController.b().onTrue(new WristToPosition(WRIST_TRAP_POSITION));
        xboxController.back().onTrue(new ArmToPosition(ARM_PRETRAP_POSITION));

        //arm and wrist switching with 
        xboxController.start().whileTrue(new InstantCommand(() -> arm.setVelocityMode(true)));
        xboxController.start().whileTrue(new InstantCommand(() -> wrist.setVelocityMode(true)));
        xboxController.start().whileFalse(new InstantCommand(() -> arm.setVelocityMode(false)));
        xboxController.start().whileFalse(new InstantCommand(() -> wrist.setVelocityMode(false)));

        xboxController.povDown().toggleOnTrue(new InstantCommand(() -> climber.setRightServo(90)));
        xboxController.povRight().toggleOnTrue(new InstantCommand(() -> climber.setRightServo(45)));
        //xboxController.povDown().toggleOnFalse(new InstantCommand(() -> climber.setRightServo(0)));
        xboxController.povDown().toggleOnTrue(new InstantCommand(() -> climber.setLeftServo(90)));
        xboxController.povRight().toggleOnTrue(new InstantCommand(() -> climber.setLeftServo(45+90)));
        //xboxController.povDown().toggleOnFalse(new InstantCommand(() -> climber.setLeftServo(0)));

        joystick.button(12).toggleOnTrue(new ClimberToPosition(LEFT_CLIMBER_REST_POSITION, RIGHT_CLIMBER_REST_POSITION));
        joystick.button(12).toggleOnTrue(SetArmAndWristPos.trap().andThen(new InstantCommand(() -> wrist.setServoPos(0))));
        joystick.button(11).toggleOnTrue(new ClimberToVelocity(zeroSupplier, zeroSupplier));
        xboxController.povUp().toggleOnTrue(new ClimberToPosition(LEFT_CLIMBER_MAX_POSITION, RIGHT_CLIMBER_MAX_POSITION));
        xboxController.leftBumper().onTrue(new WristToPosition(WRIST_REST_POSITION));
        xboxController.leftTrigger().onTrue(new WristToPosition(WRIST_INTAKE_POSITION));

       lights.setDefaultCommand(new LightsCommand());


        // climber.setDefaultCommand(new ClimberToVelocity(Control::getLeftJoystickY, Control::getRightJoystickY));

       
        

        // if(xboxController.getLeftTriggerAxis() > 0.2 && xboxController.getRightTriggerAxis() > 0.2){
        //     Commands.startEnd(() -> new ClimberToPosition(CLIMBER_MAX_POSITION), 
        //     () -> new ClimberToPosition(CLIMBER_REST_POSITION));
        // }



        // xboxController.y().toggleOnTrue(new SetArmAndWristPos(ARM_TRAP_POSITION, WRIST_TRAP_POSITION));

        
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

    public static double modifyAxisTwist(double value, double throttleValue) {
        value = deadband(value, DEAD_BAND);

        value = Math.copySign(value, value);

        throttleValue = (-throttleValue + 1) / 2;

        return value * (throttleValue * (MAX_THROTTLE - MIN_ROTATE) + MIN_ROTATE);
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
