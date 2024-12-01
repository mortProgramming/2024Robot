package org.mort11.subsystems;

import static org.mort11.config.constants.PhysicalConstants.Drivetrain.*;
import static org.mort11.config.constants.PIDConstants.Drivetrain.*;
import static org.mort11.config.constants.PortConstants.Drivetrain.*;
import static org.mort11.mortlib.hardware.encoder.EncoderTypeEnum.*;
import static org.mort11.mortlib.hardware.imu.IMUTypeEnum.*;
import static org.mort11.mortlib.hardware.motor.MotorTypeEnum.*;
import static org.mort11.mortlib.swerve.ModuleConfigEnum.*;

import org.mort11.config.IO;
import org.mort11.mortlib.subsystems.SwerveDriveBase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class Drivetrain extends SwerveDriveBase {
	private static Drivetrain drivetrain;

	private ProfiledPIDController xToPosController;
	private ProfiledPIDController yToPosController;
  	private ProfiledPIDController rotateToAngleController;

	private Drivetrain() {
		super(
			DRIVETRAIN_WHEELBASE_METERS, DRIVETRAIN_TRACKWIDTH_METERS,
			KRAKEN,
			FRONT_LEFT_DRIVE_MOTOR, FRONT_RIGHT_DRIVE_MOTOR,
			BACK_LEFT_DRIVE_MOTOR, BACK_RIGHT_DRIVE_MOTOR,
			KRAKEN,
			FRONT_LEFT_STEER_MOTOR, FRONT_RIGHT_STEER_MOTOR,
			BACK_LEFT_STEER_MOTOR, BACK_RIGHT_STEER_MOTOR,
			CANCODER,
			FRONT_LEFT_ENCODER, FRONT_RIGHT_ENCODER,
			BACK_LEFT_ENCODER, BACK_RIGHT_ENCODER,
			MK4i_L3, NAVX2,
			FRONT_LEFT_OFFSET, FRONT_RIGHT_OFFSET,
			BACK_LEFT_OFFSET, BACK_RIGHT_OFFSET
		);

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

	@Override
	public void periodic() {
		ChassisSpeeds speeds;

    	if(IO.getIsBlue()) {
			speeds = new ChassisSpeeds(
				-getChassisSpeeds().vyMetersPerSecond, -getChassisSpeeds().vxMetersPerSecond,
				getChassisSpeeds().omegaRadiansPerSecond
			);
		}
		else {
			speeds = new ChassisSpeeds(
				getChassisSpeeds().vyMetersPerSecond, getChassisSpeeds().vxMetersPerSecond,
				getChassisSpeeds().omegaRadiansPerSecond
			);
		};

		setDrive(speeds);

    	getSwerveDrive().update();
	}

	public double calculateChangeRotateController(double wantedPosition) {
		return rotateToAngleController.calculate(getIMURotation().getDegrees(), getIMURotation().getDegrees() + wantedPosition);
	}



	public ProfiledPIDController getXController() {
		return xToPosController;
	}

	public ProfiledPIDController getYController() {
		return yToPosController;
	}

	public ProfiledPIDController getRotateController() {
		return rotateToAngleController;
	}

	public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
		}
		return drivetrain;
	}
}
