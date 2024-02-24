package frc.robot.subsystems;

import static frc.robot.utility.Constants.Drivetrain.*;
import static frc.robot.utility.Constants.RobotSpecs.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Control;

public class Drivetrain extends SubsystemBase {
	private AHRS navX;
	//Declarations of necessary componenets to create drivetrain & its functions
	public final SwerveDriveKinematics driveKinematics;
	// private SwerveDriveOdometry driveOdometry;

	private final SwerveModule frontLeftModule;
	private final SwerveModule frontRightModule;
	private final SwerveModule backLeftModule;
	private final SwerveModule backRightModule;

	private double fieldOrientationOffset;

	private ChassisSpeeds chassisSpeeds;

	private PIDController aprilXController;
	private PIDController aprilYController;
	private PIDController aprilOmegaController;
	//Declarations of PID controllers for drivetrain motors
	private PIDController xController;
	private PIDController yController;
	private PIDController thetaController;

	private PIDController rotateToAngleController;
	//Declaration of drivetrain variable
	private static Drivetrain drivetrain;

	public Drivetrain() {
		navX = new AHRS(SerialPort.Port.kMXP);
		
		driveKinematics = new SwerveDriveKinematics(
				// Front left
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Front right
				new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back left
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
				// Back right
				new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		//	Puts Drivetrain on shuffleboard
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		//	Builds Front left swerve module with motors and encoders
		frontLeftModule = new MkSwerveModuleBuilder()
				.withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.FALCON, FRONT_LEFT_DRIVE)
				.withSteerMotor(MotorType.FALCON, FRONT_LEFT_STEER)
				.withSteerEncoderPort(FRONT_LEFT_STEER_ENCODER).withSteerOffset(FRONT_LEFT_STEER_OFFSET)
				.build();

		//	Builds Front Right swerve module with motors and encoders
		frontRightModule = new MkSwerveModuleBuilder()
				.withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.FALCON, FRONT_RIGHT_DRIVE)
				.withSteerMotor(MotorType.FALCON, FRONT_RIGHT_STEER)
				.withSteerEncoderPort(FRONT_RIGHT_STEER_ENCODER).withSteerOffset(FRONT_RIGHT_STEER_OFFSET)
				.build();

		//	Builds Back left swerve module with motors and encoders
		backLeftModule = new MkSwerveModuleBuilder()
				.withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.FALCON, BACK_LEFT_DRIVE)
				.withSteerMotor(MotorType.FALCON, BACK_LEFT_STEER)
				.withSteerEncoderPort(BACK_LEFT_STEER_ENCODER).withSteerOffset(BACK_LEFT_STEER_OFFSET)
				.build();

		//	Builds Back Right swerve module with motors and encoders
		backRightModule = new MkSwerveModuleBuilder()
				.withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L2)
				.withDriveMotor(MotorType.FALCON, BACK_RIGHT_DRIVE)
				.withSteerMotor(MotorType.FALCON, BACK_RIGHT_STEER)
				.withSteerEncoderPort(BACK_RIGHT_STEER_ENCODER).withSteerOffset(BACK_RIGHT_STEER_OFFSET)
				.build();
			
		//Drivetrain odometry setup
		// driveOdometry = new SwerveDriveOdometry(driveKinematics, getGyroscopeRotation(), getModulePositions());

		//	Initialization of PID controller x
		xController = new PIDController(0.8, 0, 0);
		xController.setSetpoint(-3);
		xController.setTolerance(0.1);

		//	Initialization of PID controller y
		yController = new PIDController(.41, 0, 0);
		yController.setSetpoint(0);
		yController.setTolerance(0.1);

		//	Initialization of PID controller theta
		thetaController = new PIDController(0.04, 0, 0);
		thetaController.setSetpoint(0);
		thetaController.setTolerance(0.5);

		//	Initialization of PID controller rotateToAngle
		rotateToAngleController = new PIDController(0.07, 0, 0.001);
		rotateToAngleController.setTolerance(0.5);
		// rotateToAngleController.enableContinuousInput(-180, 180);
		rotateToAngleController.enableContinuousInput(0, 360);


		aprilXController = new PIDController(XVALUE_KP , XVALUE_KI, XVALUE_KD);
		aprilXController.setTolerance(XVALUE_TOLERANCE);
		aprilYController = new PIDController(YVALUE_KP, YVALUE_KI, YVALUE_KD);
		aprilYController.setTolerance(YVALUE_TOLERANCE);
		aprilOmegaController = new PIDController(OMEGAVALUE_KP, OMEGAVALUE_KI , OMEGAVALUE_KD);
		aprilOmegaController.setTolerance(OMEGAVALUE_TOLERANCE);
	}

	/** Sets the gyroscope angle to zero. */
	// public void zeroGyroscope() {
	// 	navX.zeroYaw();
	// }
	public void zeroGyroscope(double angle) {
		fieldOrientationOffset = navX.getAngle()+angle;
	}
	public double toCircle(double angle){
		if(angle < 0){
			return angle+360;
		}else if(angle >360){
			return angle-360;
		}
		return angle;
	}

	/**
	 * Gets the current rotation of the robot from its gyroscope
	 * @return Rotation2d
	 * Rotation of the robot as a Rotation2d
	 */
	public Rotation2d getGyroscopeRotation() {
		if (navX.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(toCircle(navX.getFusedHeading()-fieldOrientationOffset));
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - toCircle(navX.getYaw()-fieldOrientationOffset));
	}

	/**
	 * @return SwerveDriveKinematics
	 */
	public SwerveDriveKinematics getDriveKinematics() {
		return driveKinematics;
	}

	/**
	 * @return Pose2d
	 */
	// public Pose2d getPose() {
	// 	return driveOdometry.getPoseMeters();
	// }

	// /**
	//  * @param pose
	//  */
	// public void resetPose(Pose2d pose) {
	// 	// driveOdometry.resetPosition(Rotation2d.fromDegrees(navX.getFusedHeading()),
	// 	// getModulePositions(),
	// 	// pose);

	// 	// //todo: try this
	// 	// /*
	// 	driveOdometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);

	// 	// */
	// }

	/**
	 * Gets the position of each swerve module
	 * @return 
	 * Each position in a SwerveModulePosition array. In order of FrontLeft, FrontRight, BackLeft, BackRight.
	 */
	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[]{frontLeftModule.getPosition(), frontRightModule.getPosition(),
				backLeftModule.getPosition(), backRightModule.getPosition()};
	}

	/**
	 * 
	 * @return
	 */
	public AHRS getNavX() {
		return navX;
	}

	/**
	 * @param states
	 */
	public void setModuleStates(SwerveModuleState[] states) {
		SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

		frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[0].angle.getRadians() + Math.toRadians(90));
		frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians() + Math.toRadians(90));
		backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians() + Math.toRadians(90));
		backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians() + Math.toRadians(90));
	}

	/**
	 * 
	 * @return
	 */
	public double getCompass() {
		return navX.getCompassHeading();
	}

	/**
	 * Set the ChassisSpeed of the swerve drive
	 * @param chassisSpeeds
	 * The Speed to set the swerve to
	 */
	public void drive(ChassisSpeeds chassisSpeeds) {
		this.chassisSpeeds = chassisSpeeds;
	}

	/**
	 * 
	 * @return
	 */
	public PIDController getXController() {
		return xController;
	}

	/**
	 * 
	 * @return
	 */
	public PIDController getYController() {
		return yController;
	}

	/**
	 * 
	 * @return
	 */
	public PIDController getThetaController() {
		return thetaController;
	}

	/**
	 * 
	 * @return
	 */
	public PIDController getRotateToAngleController() {
		return rotateToAngleController;
	}

	public PIDController getAprilTagXController() {
		return aprilXController;
	}

	public PIDController getAprilTagYController() {
		return aprilYController;
	}

	public PIDController getAprilTagOmegaController() {
		return aprilOmegaController;
	}

	/**
	 * 
	 */
	@Override
	public void periodic() {
		// driveOdometry.update(Rotation2d.fromDegrees(navX.getFusedHeading()), getModulePositions());

		SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(chassisSpeeds);
		setModuleStates(states);

		SmartDashboard.putNumber("Angle", getGyroscopeRotation().getDegrees());
		SmartDashboard.putNumber("ThrottleThing", Control.getThrottle());

	}

	/**
	 * Returns an instance of drivetrain. Use this method when creating a drivetrain object anywhere in the code, do NOT use constructor
	 * @return A Drivetrain instance. If one doesn't exist already, it creates one.
	 */
	public static Drivetrain getInstance() {
		if (drivetrain == null) {
			drivetrain = new Drivetrain();
			Shuffleboard.getTab("dt").add(drivetrain);

		}
		return drivetrain;
	}

	public double getPitch(){
		return navX.getPitch() - 0.3;
	}

	public double getRoll(){
		return navX.getRoll() + 1;
	}

}

