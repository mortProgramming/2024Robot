package org.mort11.config;

import org.mort11.subsystems.Arm;
import org.mort11.subsystems.Climber;
import org.mort11.subsystems.Drivetrain;
import org.mort11.subsystems.Intake;
import org.mort11.subsystems.Lights;
import com.LimelightHelpers.*;
import org.mort11.subsystems.Wrist;

import static org.mort11.config.Inputs.*;
import static org.mort11.config.constants.PhysicalConstants.Arm.*;
import static org.mort11.config.constants.PhysicalConstants.Climber.*;
import static org.mort11.config.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.config.constants.PhysicalConstants.Intake.*;
import static org.mort11.config.constants.PhysicalConstants.Wrist.*;
import static org.mort11.config.constants.PortConstants.Vision.*;

import org.mort11.commands.actions.drivetrain.Drive;
import org.mort11.commands.actions.drivetrain.DriveAtAngle;
import org.mort11.commands.actions.drivetrain.DriveNoteLocked;
import org.mort11.commands.actions.endeffector.IntakeRest;
import org.mort11.commands.actions.endeffector.Lighting;
import org.mort11.commands.actions.endeffector.pos.ClimberToPos;
import org.mort11.commands.actions.endeffector.pos.SetArmWristPos;
import org.mort11.commands.actions.endeffector.pos.WristToPos;
import org.mort11.commands.actions.endeffector.vel.BlowerToVel;
import org.mort11.commands.actions.endeffector.vel.ClimberToVel;
import org.mort11.commands.actions.endeffector.vel.IntakeToVel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IO {

	private static Drivetrain drivetrain;
    private static Arm arm;
    private static Climber climber;
    private static Wrist wrist;
    private static Lights lights;
    private static Intake intake;

    public static void init() {
		drivetrain = Drivetrain.getInstance();
        arm = Arm.getInstance();
        wrist = Wrist.getInstance();
        climber = Climber.getInstance();
        intake = Intake.getInstance();
        lights = Lights.getInstance();
        System.out.println("Subsystem init");
    }

    public static void configure() {
        init();
        Inputs.init();

		drivetrain.setDefaultCommand(
			new Drive(Inputs::getJoystickX, Inputs::getJoystickY, Inputs::getJoystickRotate, true)
        );

        lights.setDefaultCommand(new Lighting());

       //Drivetrain Field Orient command
        joystick.button(2).whileTrue(drivetrain.setGyroscopeZero(0));

        //Drivetrain note locking command
        joystick.trigger().whileTrue(new DriveNoteLocked(Inputs::getJoystickY, Inputs::getJoystickX));

        //Drivetrain reset odometry command
        joystick.button(7).whileTrue(Odometer.resetOdometryCommand(LimelightHelpers.getBotPose2d_wpiBlue(TAG_CAMERA)));

        //Drivetrain rotate to AMP button
        joystick.button(3).whileTrue(new DriveAtAngle(Inputs::getJoystickY, Inputs::getJoystickX, IMU_TO_ROBOT_FRONT_ANGLE));



        // Basics
        xboxController.rightBumper().whileTrue(new IntakeRest());
        xboxController.leftBumper().onTrue(new WristToPos(WRIST_REST_POS));
        xboxController.leftTrigger().onTrue(new WristToPos(WRIST_INTAKE_POS));

        xboxController.rightTrigger().whileTrue(new IntakeToVel(AMP_SHOOT_SPEED));
        xboxController.a().whileTrue(new IntakeToVel(SHOOTER_SHOOT_SPEED));
        
        xboxController.x().onTrue(SetArmWristPos.amp());
        xboxController.y().onTrue(SetArmWristPos.rest());
        xboxController.b().onTrue(SetArmWristPos.trap());

        //ARM TO PRETRAP
        xboxController.back().onTrue(SetArmWristPos.preTrap());
        xboxController.back().onTrue(new BlowerToVel(BLOWER_MOTOR_MAX_SPEED));

        //floor trap
        xboxController.povDown().whileTrue(new BlowerToVel(BLOWER_MOTOR_MAX_SPEED));
        xboxController.povDown().whileTrue(SetArmWristPos.floorTrap()
            .andThen(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_POS))));

        xboxController.povDown().onFalse(new BlowerToVel(0));
        xboxController.povDown().whileFalse(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_REST_POS)));
        
        //Climbers up for preclimb
        xboxController.povUp().toggleOnTrue(new ClimberToPos(LEFT_CLIMBER_MAX_POS, RIGHT_CLIMBER_MAX_POS));

        //TRAP CLIMB
        joystick.button(12).toggleOnTrue(new ClimberToPos(LEFT_CLIMBER_REST_POS, RIGHT_CLIMBER_REST_POS));
        joystick.button(12).toggleOnTrue(SetArmWristPos.trap().andThen(new InstantCommand(() -> wrist.setServoPos(TRAP_SERVO_POS))));
        joystick.button(11).onTrue(new ClimberToVel(0, 0));

        //MANUAL CLIMBER CONTROL
        xboxController.povLeft().whileTrue(new ClimberToVel(MANUAL_CLIMBER_SPEED, 0));
        // opposite direction for opposite side
        xboxController.povRight().whileTrue(new ClimberToVel(0, -MANUAL_CLIMBER_SPEED));
    }

    public static Boolean isBlue() {
		return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true;
	}
 
 }
