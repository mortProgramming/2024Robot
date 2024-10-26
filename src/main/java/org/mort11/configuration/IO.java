package org.mort11.configuration;

import org.mort11.commands.Actions.Drivetrain.MoveToAprilTag;
import org.mort11.commands.Actions.EndEffector.BlowerToVelocity;
import org.mort11.commands.Actions.EndEffector.ClimberToPosition;
import org.mort11.commands.Actions.EndEffector.ClimberToVelocity;
import org.mort11.commands.Actions.EndEffector.IntakeBeamBreak;
import org.mort11.commands.Actions.EndEffector.IntakeToVelocity;
import org.mort11.commands.Actions.EndEffector.LightsCommand;
import org.mort11.commands.Actions.EndEffector.ArmWrist.ArmToPosition;
import org.mort11.commands.Actions.EndEffector.ArmWrist.SetArmAndWristPos;
import org.mort11.commands.Actions.EndEffector.ArmWrist.WristToPosition;
import org.mort11.commands.Teleop.DrivetrainCommand;
import org.mort11.subsystems.Arm;
import org.mort11.subsystems.Climber;
import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Lights;
import org.mort11.subsystems.Vision;
import org.mort11.subsystems.Wrist;

import static org.mort11.configuration.Inputs.*;
import static org.mort11.configuration.Constants.Arm.*;
import static org.mort11.configuration.Constants.Climber.*;
import static org.mort11.configuration.Constants.Intake.*;
import static org.mort11.configuration.Constants.Wrist.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IO {
    private static DoubleSupplier zeroSupplier = new DoubleSupplier() {
        public double getAsDouble() {
            return 0.0;
        }
    };

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Climber climber;
    private static Wrist wrist;
    private static Vision vision;
    private static Lights lights;
    private static Intake intake;

    public static void init() {

		drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();
        vision = Vision.getInstance();
        lights = Lights.getInstance();
        System.out.println("Subsystem init");

        PathAuto.init();//Drivetrain methods must properly exist for the PathPlanner swerve configuration to work.
        System.out.println("auto init");
    }

    public static void configure() {
        init();
        Inputs.init();

		drivetrain.setDefaultCommand(
			new DrivetrainCommand(Inputs::getJoystickY, Inputs::getJoystickX, Inputs::getJoystickTwist, true)
        );

       //Drivetrain Field Orient command
        joystick.button(2).whileTrue(new InstantCommand(() -> drivetrain.zeroGyroscope(0)));

        //Drivetrain note locking command
        joystick.trigger().whileTrue(new InstantCommand(() -> drivetrain.noteLockOn()));
        joystick.trigger().whileFalse(new InstantCommand(() -> drivetrain.noteLockOff()));

        //Drivetrain reset odometry command
        joystick.button(7).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(vision.getFieldPose())));

        //Drivetrain rotate to AMP button (NOT WORKING RED/BLUE)
        joystick.button(9).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(3).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(9).or(joystick.button(3)).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(9).whileTrue(new InstantCommand(() -> drivetrain.setKeptAngleRelative(90)));
        joystick.button(9).onFalse(new InstantCommand(() -> drivetrain.setIsAngleKept(false)));
        joystick.button(3).onFalse(new InstantCommand(() -> drivetrain.setIsAngleKept(false)));

        // joystick.button(3).whileTrue(
        //     new InstantCommand(() -> drivetrain.setKeptAngle(
        //         Drivetrain.toCircle(
        //             180 + drivetrain.getGyroscopeRotation().getDegrees() + vision.getNoteXDegrees())
        //         )
        //     )
        // );
        
        //Odometry reset to robot starting position command
        joystick.button(8).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(new Pose2d(1.515,7.395, Rotation2d.fromRadians(-1.571)))));

        //Move to april tag 15 (not tested)
        joystick.button(6).whileTrue(new MoveToAprilTag(15));

      //  joystick.button(5).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(0.4, 7.5, 90)));

        //Wrist intaking (USING INTAKE BEAM BREAK)
        xboxController.rightBumper().whileTrue(new IntakeBeamBreak(WRIST_REST_POSITION));

        //Wrist outtaking (shoot into amp)
        xboxController.rightTrigger().onTrue(new IntakeToVelocity(AMP_SHOOT_SPEED));
        xboxController.rightTrigger().onFalse(new IntakeToVelocity(0));

        //Wrist outtaking (shoot into trap/FULL SPEED)
        xboxController.a().onTrue(new IntakeToVelocity(SHOOTER_SHOOT_SPEED));
        xboxController.a().onFalse(new IntakeToVelocity(0));
        
        //ARM TO AMP
        xboxController.x().onTrue(new ArmToPosition(ARM_AMP_POSITION));

        //ARM TO REST
        xboxController.y().onTrue(new ArmToPosition(ARM_REST_POSITION));

        //ARM AND WRIST TO TRAP
        xboxController.b().onTrue(new ArmToPosition(ARM_TRAP_POSITION));
        xboxController.b().onTrue(new WristToPosition(WRIST_TRAP_POSITION));

        //ARM TO PRETRAP
        xboxController.back().onTrue(new ArmToPosition(ARM_PRETRAP_POSITION).andThen(new BlowerToVelocity(BLOWER_MOTOR_MAX_SPEED)));

        //arm and wrist switching with 
        xboxController.start().whileTrue(new InstantCommand(() -> arm.setVelocityMode(true)));
        xboxController.start().whileTrue(new InstantCommand(() -> wrist.setVelocityMode(true)));
        xboxController.start().whileFalse(new InstantCommand(() -> arm.setVelocityMode(false)));
        xboxController.start().whileFalse(new InstantCommand(() -> wrist.setVelocityMode(false)));

        //floor trap
        xboxController.povDown().whileTrue(new BlowerToVelocity(-BLOWER_MOTOR_MAX_SPEED));
        xboxController.povDown().onFalse(new BlowerToVelocity(0).andThen(new InstantCommand(() -> wrist.setServoPos(90))));
    
        xboxController.povDown().whileFalse(new InstantCommand(() -> wrist.setServoPos(90)));
        xboxController.povDown().whileTrue(SetArmAndWristPos.floorTrap().andThen(new InstantCommand(() -> {wrist.setServoPos(TRAP_SERVO_POS);})));

        //Climber to trap/Arm and Wrist to trap
        joystick.button(12).toggleOnTrue(new ClimberToPosition(LEFT_CLIMBER_REST_POSITION, RIGHT_CLIMBER_REST_POSITION));
        joystick.button(12).toggleOnTrue(SetArmAndWristPos.trap().andThen(new InstantCommand(() -> wrist.setServoPos(0))));
        joystick.button(11).toggleOnTrue(new ClimberToVelocity(zeroSupplier, zeroSupplier));
        
        //Climbers up for preclimb
        xboxController.povUp().toggleOnTrue(new ClimberToPosition(LEFT_CLIMBER_MAX_POSITION, RIGHT_CLIMBER_MAX_POSITION));
        //Wrist to rest
        xboxController.leftBumper().onTrue(new WristToPosition(WRIST_REST_POSITION));

        //Wrist to intake
        xboxController.leftTrigger().onTrue(new WristToPosition(WRIST_INTAKE_POSITION));

        //MANUAL CLIMBER CONTROL
        xboxController.povLeft().whileTrue(new ClimberToVelocity(() -> {return 1;} ,() -> {return 0;}));
        xboxController.povRight().whileTrue(new ClimberToVelocity(() -> {return 0;} ,() -> {return -1;}));

        lights.setDefaultCommand(new LightsCommand());
    }

    public static Boolean isBlue() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}
 
 }
