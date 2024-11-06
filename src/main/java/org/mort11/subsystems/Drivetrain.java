package org.mort11.subsystems;

import org.mort11.configuration.Odometer;
import org.mort11.configuration.IO;
import org.mort11.library.Hardware.IMU.IMU;
import org.mort11.library.Swerve.SwerveModule;
import org.mort11.library.Swerve.SwerveDrives.SwerveDrive;

import static org.mort11.configuration.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PIDConstants.Drivetrain.*;
import static org.mort11.configuration.constants.PortConstants.Drivetrain.*;
import static org.mort11.library.Hardware.Encoder.EncoderTypeEnum.*;
import static org.mort11.library.Hardware.IMU.IMUTypeEnum.*;
import static org.mort11.library.Hardware.Motor.MotorTypeEnum.*;
import static org.mort11.library.Swerve.ModuleTypeEnum.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
	private static Drivetrain drivetrain;

	private SwerveDrive swerveDrive;

	private SwerveModule frontLeftModule;
	private SwerveModule frontRightModule;
	private SwerveModule backLeftModule;
	private SwerveModule backRightModule;

	private SwerveDriveKinematics driveKinematics;

	private IMU imu;

	private ChassisSpeeds speeds;

	private double fieldOrientationOffset;

	private ProfiledPIDController xToPosController;
	private ProfiledPIDController yToPosController;
	private ProfiledPIDController rotateToAngleController;

	private Drivetrain() {
		imu = new IMU(NAVX2, 0);

		configureSwerve();

		speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		fieldOrientationOffset = 0;

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
		frontLeftModule = new SwerveModule(
			KRAKEN, FRONT_LEFT_DRIVE_MOTOR, 
			KRAKEN, FRONT_LEFT_STEER_MOTOR, 
			CANCODER, FRONT_LEFT_ENCODER, 
			MK4i
		);
			
		frontRightModule = new SwerveModule(
			KRAKEN, FRONT_RIGHT_DRIVE_MOTOR, 
			KRAKEN, FRONT_RIGHT_STEER_MOTOR, 
			CANCODER, FRONT_RIGHT_ENCODER, 
			MK4i
		);
			
		backLeftModule = new SwerveModule(
			KRAKEN, BACK_LEFT_DRIVE_MOTOR, 
			KRAKEN, BACK_LEFT_STEER_MOTOR, 
			CANCODER, BACK_LEFT_ENCODER, 
			MK4i
		);
			
		backRightModule = new SwerveModule(
			KRAKEN, BACK_RIGHT_DRIVE_MOTOR, 
			KRAKEN, BACK_RIGHT_STEER_MOTOR, 
			CANCODER, BACK_RIGHT_ENCODER, 
			MK4i
		);

		frontLeftModule.steerMotor.setDirectionFlip(true);
    	frontRightModule.steerMotor.setDirectionFlip(true);
    	backLeftModule.steerMotor.setDirectionFlip(true);
    	backRightModule.steerMotor.setDirectionFlip(true);

		driveKinematics = new SwerveDriveKinematics(
			// Front left
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Front right
			new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back left
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
			// Back right
			new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
		);

		swerveDrive = new SwerveDrive(
			frontLeftModule, frontRightModule,
			backLeftModule, backRightModule,
			driveKinematics
		);

		swerveDrive.setOffsets(FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET, BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET);
	}

	@Override
	public void periodic() {
		if (IO.isBlue()) {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				speeds.vyMetersPerSecond,-speeds.vxMetersPerSecond,
				speeds.omegaRadiansPerSecond, 
				drivetrain.getIMURotation()
			);
		}
		else {
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
				-speeds.vyMetersPerSecond,speeds.vxMetersPerSecond,
				speeds.omegaRadiansPerSecond, 
				drivetrain.getIMURotation()
			);
		}

		swerveDrive.setVelocity(speeds);

		SmartDashboard.putNumber("Angle", getIMURotation().getDegrees());
		SmartDashboard.putNumber("Other angle", imu.getAngle());
	}

	public void setDrive(ChassisSpeeds speeds) {
		this.speeds = speeds;
	}

	public void setPosController(double poseX, double poseY, double wantedX, double wantedY) {
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			xToPosController.calculate(Odometer.getPoseX(), wantedX), 
        	yToPosController.calculate(Odometer.getPoseY(), wantedY), 
        	0,
			drivetrain.getIMURotation()
		);
	}

	public void setAngleController(double wantedAngle) {
		speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			speeds.vxMetersPerSecond, 
        	speeds.vyMetersPerSecond,
			rotateToAngleController.calculate(drivetrain.getIMURotation().getDegrees(), wantedAngle),
			drivetrain.getIMURotation()
		);
	}

	public Command setGyroscopeZero(double angle) {
		// return new InstantCommand(() -> swerveDrive.zeroIMU(angle));
		return new InstantCommand(() -> fieldOrientationOffset = getIMURotation().getDegrees() + angle);
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
        return speeds;
    }

	public double getMaxSpeedMeters() {
		return frontLeftModule.maxSpeed;
	}
	
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public SwerveDriveKinematics getDriveKinematics() {
		return driveKinematics;
	}
	
	public Rotation2d getIMURotation() {
		return getAbsoluteIMURotation().minus(Rotation2d.fromDegrees(fieldOrientationOffset));
	}

	public Rotation2d getAbsoluteIMURotation() {
		return Rotation2d.fromDegrees(imu.getAngle());
	}

	public Rotation2d getRotation2d() {
		return imu.getRotation2d();
	}

	public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}
}
