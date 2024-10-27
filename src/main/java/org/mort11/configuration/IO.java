package org.mort11.configuration;

// import org.mort11.subsystems.Arm;
// import org.mort11.subsystems.Climber;
import org.mort11.subsystems.Drivetrain;
// import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Lights;
import org.mort11.subsystems.LimelightHelpers;
import org.mort11.subsystems.Wrist;

import static org.mort11.configuration.Inputs.*;
import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PhysicalConstants.Climber.*;
import static org.mort11.configuration.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PhysicalConstants.Intake.*;
import static org.mort11.configuration.constants.PhysicalConstants.Wrist.*;
import static org.mort11.configuration.constants.PortConstants.Vision.*;

import org.mort11.commands.actions.drivetrain.Drive;
import org.mort11.commands.actions.endeffector.BlowerToVelocity;
import org.mort11.commands.actions.endeffector.ClimberToPos;
import org.mort11.commands.actions.endeffector.ClimberToVelocity;
import org.mort11.commands.actions.endeffector.IntakeBeamBreak;
import org.mort11.commands.actions.endeffector.IntakeToVelocity;
import org.mort11.commands.actions.endeffector.LightsCommand;
import org.mort11.commands.actions.endeffector.armwrist.SetArmAndWristPos;
import org.mort11.commands.actions.endeffector.armwrist.WristToPos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IO {

	private static Drivetrain drivetrain;
    // private static Arm arm;
    // private static Climber climber;
    private static Wrist wrist;
    private static Lights lights;
    // private static Intake intake;

    public static void init() {

		drivetrain = Drivetrain.getInstance();
        // arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        // climber = Climber.getInstance();
        // intake = Intake.getInstance();
        lights = Lights.getInstance();
        System.out.println("Subsystem init");
    }

    public static void configure() {
        init();
        Inputs.init();

		drivetrain.setDefaultCommand(
			new Drive(Inputs::getJoystickY, Inputs::getJoystickX, Inputs::getJoystickRotate, true)
        );

        lights.setDefaultCommand(new LightsCommand());

       //Drivetrain Field Orient command
        joystick.button(2).whileTrue(new InstantCommand(() -> drivetrain.zeroGyroscope(0)));

        //Drivetrain note locking command
        joystick.trigger().whileTrue(new InstantCommand(() -> drivetrain.noteLockOn()));
        joystick.trigger().whileFalse(new InstantCommand(() -> drivetrain.noteLockOff()));

        //Drivetrain reset odometry command
        joystick.button(7).whileTrue(new InstantCommand(() -> Odometer.resetOdometry(LimelightHelpers.getBotPose2d_wpiBlue(TAG_CAMERA))));

        //Drivetrain rotate to AMP button (NOT WORKING RED/BLUE)
        joystick.button(3).whileTrue(new InstantCommand(() -> drivetrain.setIsAngleKept(true)));
        joystick.button(3).whileTrue(new InstantCommand(() -> drivetrain.setKeptAngleRelative(IMU_TO_ROBOT_FRONT_ANGLE)));
        joystick.button(3).onFalse(new InstantCommand(() -> drivetrain.setIsAngleKept(false)));



        // Basics
        xboxController.rightBumper().whileTrue(new IntakeBeamBreak(WRIST_REST_POS));
        xboxController.leftBumper().onTrue(new WristToPos(WRIST_REST_POS));
        xboxController.leftTrigger().onTrue(new WristToPos(WRIST_INTAKE_POS));

        xboxController.rightTrigger().whileTrue(new IntakeToVelocity(AMP_SHOOT_SPEED));
        xboxController.a().whileTrue(new IntakeToVelocity(SHOOTER_SHOOT_SPEED));
        
        xboxController.x().onTrue(SetArmAndWristPos.amp());
        xboxController.y().onTrue(SetArmAndWristPos.rest());
        xboxController.b().onTrue(SetArmAndWristPos.trap());

        //ARM TO PRETRAP
        xboxController.back().onTrue(SetArmAndWristPos.preTrap());
        xboxController.back().onTrue(new BlowerToVelocity(BLOWER_MOTOR_MAX_SPEED));

        //floor trap
        xboxController.povDown().whileTrue(new BlowerToVelocity(BLOWER_MOTOR_MAX_SPEED));
        xboxController.povDown().whileTrue(SetArmAndWristPos.floorTrap()
            .andThen(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_POS))));

        xboxController.povDown().onFalse(new BlowerToVelocity(0));
        xboxController.povDown().whileFalse(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_REST_POS)));
        
        //Climbers up for preclimb
        xboxController.povUp().toggleOnTrue(new ClimberToPos(LEFT_CLIMBER_MAX_POS, RIGHT_CLIMBER_MAX_POS));

        //TRAP CLIMB
        joystick.button(12).toggleOnTrue(new ClimberToPos(LEFT_CLIMBER_REST_POS, RIGHT_CLIMBER_REST_POS));
        joystick.button(12).toggleOnTrue(SetArmAndWristPos.trap().andThen(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_POS))));
        joystick.button(11).onTrue(new ClimberToVelocity(0, 0));

        //MANUAL CLIMBER CONTROL
        xboxController.povLeft().whileTrue(new ClimberToVelocity(MANUAL_CLIMBER_SPEED, 0));
        // opposite direction for opposite side
        xboxController.povRight().whileTrue(new ClimberToVelocity(0, -MANUAL_CLIMBER_SPEED));
    }

    public static Boolean isBlue() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}
 
 }
