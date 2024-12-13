package org.mort11.subsystems;

import org.mort11.configuration.Odometer;

import static org.mort11.configuration.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PIDConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PortConstants.Drivetrain.*;

import org.mort11.configuration.IO;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
	private static Drivetrain drivetrain;

	private SwerveModule frontLeftModule;
	private SwerveModule frontRightModule;
	private SwerveModule backLeftModule;
	private SwerveModule backRightModule;
	private SwerveDriveKinematics driveKinematics;

	private AHRS navX;

	private ChassisSpeeds chassisSpeeds;

	private double fieldOrientationOffset;

	private ProfiledPIDController xToPosController;
	private ProfiledPIDController yToPosController;
	private ProfiledPIDController rotateToAngleController;

	private MkModuleConfiguration defaultDriveConfig;

	private ShuffleboardTab tab;

	public Drivetrain() {
		navX = new AHRS(SPI.Port.kMXP);

		tab = Shuffleboard.getTab("Drivetrain");

		configureSwerve();

		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		xToPosController = new ProfiledPIDController(
			TO_POS_KP, TO_POS_KI, TO_POS_KD, TO_POS_CONSTRAINTS
		);
		yToPosController = new ProfiledPIDController(
			TO_POS_KP, TO_POS_KI, TO_POS_KD, TO_POS_CONSTRAINTS
		);
		rotateToAngleController = new ProfiledPIDController(
			TO_ANGLE_KP, TO_ANGLE_KI, TO_ANGLE_KD, TO_ANGLE_CONSTRAINTS
		);

		xToPosController.setTolerance(TO_POS_POS_TOLERANCE);
		yToPosController.setTolerance(TO_POS_POS_TOLERANCE);
		rotateToAngleController.setTolerance(TO_ANGLE_POS_TOLERANCE, TO_ANGLE_VEL_TOLERANCE);

		rotateToAngleController.enableContinuousInput(-180, 180);
	}

	public void configureSwerve() {
		defaultDriveConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
		defaultDriveConfig.setDriveCurrentLimit(360);
		defaultDriveConfig.setSteerCurrentLimit(360);
			
		driveKinematics = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

			//	Builds Front left swerve module with motors and encoders
			frontLeftModule = new MkSwerveModuleBuilder()
			// frontLeftModule = new MkSwerveModuleBuilder(defaultDriveConfig)
					.withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
					.withGearRatio(SdsModuleConfigurations.MK4I_L3)
					.withDriveMotor(MotorType.FALCON, FRONT_LEFT_DRIVE_MOTOR)
					.withSteerMotor(MotorType.FALCON, FRONT_LEFT_STEER_MOTOR)
					.withSteerEncoderPort(FRONT_LEFT_ENCODER).withSteerOffset(FRONT_LEFT_OFFSET)
					.build();

			//	Builds Front Right swerve module with motors and encoders
			frontRightModule = new MkSwerveModuleBuilder()
			// frontRightModule = new MkSwerveModuleBuilder(defaultDriveConfig)
					.withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0))
					.withGearRatio(SdsModuleConfigurations.MK4I_L3)
					.withDriveMotor(MotorType.FALCON, FRONT_RIGHT_DRIVE_MOTOR)
					.withSteerMotor(MotorType.FALCON, FRONT_RIGHT_STEER_MOTOR)
					.withSteerEncoderPort(FRONT_RIGHT_ENCODER).withSteerOffset(FRONT_RIGHT_OFFSET)
					.build();

			//	Builds Back left swerve module with motors and encoders
			backLeftModule = new MkSwerveModuleBuilder()
			// backLeftModule = new MkSwerveModuleBuilder(defaultDriveConfig)
					.withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
					.withGearRatio(SdsModuleConfigurations.MK4I_L3)
					.withDriveMotor(MotorType.FALCON, BACK_LEFT_DRIVE_MOTOR)
					.withSteerMotor(MotorType.FALCON, BACK_LEFT_STEER_MOTOR)
					.withSteerEncoderPort(BACK_LEFT_ENCODER).withSteerOffset(BACK_LEFT_OFFSET)
					.build();

			//	Builds Back Right swerve module with motors and encoders
			backRightModule = new MkSwerveModuleBuilder()
			// backRightModule = new MkSwerveModuleBuilder(defaultDriveConfig)
					.withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
					.withGearRatio(SdsModuleConfigurations.MK4I_L3)
					.withDriveMotor(MotorType.FALCON, BACK_RIGHT_DRIVE_MOTOR)
					.withSteerMotor(MotorType.FALCON, BACK_RIGHT_STEER_MOTOR)
					.withSteerEncoderPort(BACK_RIGHT_ENCODER).withSteerOffset(BACK_RIGHT_OFFSET)
					.build();
			
		frontLeftModule.getDriveMotor().setInverted(false);
		frontRightModule.getDriveMotor().setInverted(false);
		backLeftModule.getDriveMotor().setInverted(false);
		backRightModule.getDriveMotor().setInverted(false);
	}

	@Override
	public void periodic() {
		if (IO.isBlue()) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				chassisSpeeds.vyMetersPerSecond,-chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.omegaRadiansPerSecond, 
				drivetrain.getGyroscopeRotation()
			);
		}
		else {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				-chassisSpeeds.vyMetersPerSecond,chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.omegaRadiansPerSecond, 
				drivetrain.getGyroscopeRotation()
			);
		}

		chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
		
		setModuleStates(driveKinematics.toSwerveModuleStates(chassisSpeeds));

		SmartDashboard.putNumber("Angle", getGyroscopeRotation().getDegrees());
		SmartDashboard.putNumber("Other angle", navX.getYaw());
	}

	public void setDrive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
	}

	public void setPosController(double poseX, double poseY, double wantedX, double wantedY) {
		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			xToPosController.calculate(Odometer.getPoseX(), wantedX), 
        	yToPosController.calculate(Odometer.getPoseY(), wantedY), 
        	0,
			drivetrain.getGyroscopeRotation()
		);
	}

	public void setAngleController(double wantedAngle) {
		chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			chassisSpeeds.vxMetersPerSecond, 
        	chassisSpeeds.vyMetersPerSecond,
			rotateToAngleController.calculate(drivetrain.getGyroscopeRotation().getDegrees(), wantedAngle),
			drivetrain.getGyroscopeRotation()
		);
	}

    public void setAngle2Controller(double wantedAngle) {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                rotateToAngleController.calculate(
                        wantedAngle, 0),
                drivetrain.getGyroscopeRotation());
    }

	private void setModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12,
				states[0].angle.getRadians());
		frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12,
				states[1].angle.getRadians());
		backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12,
				states[2].angle.getRadians());
		backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * 12,
				states[3].angle.getRadians());
	}

	public Command setGyroscopeZero(double angle) {
		return new InstantCommand(() -> fieldOrientationOffset = navX.getAngle() + angle);
	}



	public boolean getXControllerAtSetpoint() {
		return xToPosController.atSetpoint();
	}

	public boolean getYControllerAtSetpoint() {
		return yToPosController.atSetpoint();
	}

	public boolean getRotateControllerAtSetpoint() {
		return rotateToAngleController.atSetpoint();
	}

	public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }
	
	//Each position in a SwerveModulePosition array. In order of FrontLeft, FrontRight, BackLeft, BackRight
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[]{frontLeftModule.getPosition(), frontRightModule.getPosition(),
				backLeftModule.getPosition(), backRightModule.getPosition()};
	}

	public SwerveDriveKinematics getDriveKinematics() {
		return driveKinematics;
	}
	
	public Rotation2d getGyroscopeRotation() {
		return getAbsoluteGyroscopeRotation().minus(Rotation2d.fromDegrees(fieldOrientationOffset));
	}

	public Rotation2d getAbsoluteGyroscopeRotation() {
		if (navX.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(360.0 - navX.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		return Rotation2d.fromDegrees(360.0 - toCircle(navX.getYaw()));
	}

	public Rotation2d getRotation2d() {
		return navX.getRotation2d();
	}



	public static double toCircle(double angle){
		if(angle < 0) {
			return angle + 360;
		} else if(angle > 360) {
			return angle - 360;
		}
		return angle;
	}

	public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
            Shuffleboard.getTab("dt").add(drivetrain);
		}
		return drivetrain;
	}
}
