package frc.robot.subsystems;

import static frc.robot.utility.Constants.Drivetrain.*;
import static frc.robot.utility.Constants.RobotSpecs.*;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.Auto;
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
	private boolean isAngleKept;
	private double setKeptAngle;
	private boolean isBlue;
	private boolean noteLock = false;
	private boolean canLock = false;
	private double ampAngleSpeed = 0;
	private Intake intake = Intake.getInstance();

	private PIDController aprilXController;
	private PIDController aprilYController;
	private PIDController aprilOmegaController;
	//Declarations of PID controllers for drivetrain motors
	private PIDController xController;
	private PIDController yController;
	private PIDController thetaController;

	private ProfiledPIDController xToPositioController;
	private ProfiledPIDController yToPositioController;
	private PIDController rotateToAngleController;
	//Declaration of drivetrain variable
	private static Drivetrain drivetrain;
	private MkModuleConfiguration defaultDriveConfig;

	public Drivetrain() {
		navX = new AHRS(SPI.Port.kMXP);
		// navX = new AHRS(I2C.Port.kMXP);
		// defaultDriveConfig = new MkModuleConfiguration();
		// defaultDriveConfig.setDriveCurrentLimit(Double.NaN);

		//defaultDriveConfig = new MkModuleConfiguration();
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

		chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

		//	Puts Drivetrain on shuffleboard
		ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

		//	Builds Front left swerve module with motors and encoders
		frontLeftModule = new MkSwerveModuleBuilder()
		// frontLeftModule = new MkSwerveModuleBuilder(defaultDriveConfig)
				.withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L3)
				.withDriveMotor(MotorType.FALCON, FRONT_LEFT_DRIVE)
				.withSteerMotor(MotorType.FALCON, FRONT_LEFT_STEER)
				.withSteerEncoderPort(FRONT_LEFT_STEER_ENCODER).withSteerOffset(FRONT_LEFT_STEER_OFFSET)
				.build();

		//	Builds Front Right swerve module with motors and encoders
		frontRightModule = new MkSwerveModuleBuilder()
		// frontRightModule = new MkSwerveModuleBuilder(defaultDriveConfig)
				.withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L3)
				.withDriveMotor(MotorType.FALCON, FRONT_RIGHT_DRIVE)
				.withSteerMotor(MotorType.FALCON, FRONT_RIGHT_STEER)
				.withSteerEncoderPort(FRONT_RIGHT_STEER_ENCODER).withSteerOffset(FRONT_RIGHT_STEER_OFFSET)
				.build();

		//	Builds Back left swerve module with motors and encoders
		backLeftModule = new MkSwerveModuleBuilder()
		// backLeftModule = new MkSwerveModuleBuilder(defaultDriveConfig)
				.withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L3)
				.withDriveMotor(MotorType.FALCON, BACK_LEFT_DRIVE)
				.withSteerMotor(MotorType.FALCON, BACK_LEFT_STEER)
				.withSteerEncoderPort(BACK_LEFT_STEER_ENCODER).withSteerOffset(BACK_LEFT_STEER_OFFSET)
				.build();

		//	Builds Back Right swerve module with motors and encoders
		backRightModule = new MkSwerveModuleBuilder()
		// backRightModule = new MkSwerveModuleBuilder(defaultDriveConfig)
				.withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0))
				.withGearRatio(SdsModuleConfigurations.MK4I_L3)
				.withDriveMotor(MotorType.FALCON, BACK_RIGHT_DRIVE)
				.withSteerMotor(MotorType.FALCON, BACK_RIGHT_STEER)
				.withSteerEncoderPort(BACK_RIGHT_STEER_ENCODER).withSteerOffset(BACK_RIGHT_STEER_OFFSET)
				.build();
		
		frontLeftModule.getDriveMotor().setInverted(false);
		frontRightModule.getDriveMotor().setInverted(false);
		backLeftModule.getDriveMotor().setInverted(false);
		backRightModule.getDriveMotor().setInverted(false);

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

		xToPositioController = new ProfiledPIDController(TO_POSITION_KP, TO_POSITION_KI, TO_POSITION_KD,
		new Constraints(TO_POSITION_KV, TO_POSITION_KA));
		xToPositioController.setTolerance(TO_POSITION_TOLERANCE);

		yToPositioController = new ProfiledPIDController(TO_POSITION_KP, TO_POSITION_KI, TO_POSITION_KD,
		new Constraints(TO_POSITION_KV, TO_POSITION_KA));
		yToPositioController.setTolerance(TO_POSITION_TOLERANCE);


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
	public void noteLockOn(){
		noteLock = true;
	}
	public void noteLockOff(){
		noteLock = false;
	}

	/** Sets the gyroscope angle to zero. 
	 * @param angle
	 * 
	*/
	public void zeroGyroscope(double angle) {
		fieldOrientationOffset = navX.getAngle()+angle;
	}
	// public double toCircle(double angle){
	// 	if(angle < 0){
	// 		return angle+360;
	// 	}else if(angle >360){
	// 		return angle-360;
	// 	}
	// 	return angle;
	// }

	public static double toCircle(double angle){
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
			return Rotation2d.fromDegrees(360.0 - (navX.getFusedHeading()-fieldOrientationOffset));
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - navX.getYaw()-fieldOrientationOffset);
	}

	/**
	 * Gets the current rotation of the robot from its gyroscope
	 * @return Rotation2d
	 * Rotation of the robot as a Rotation2d
	 */
	public Rotation2d getAbsoluteGyroscopeRotation() {
		if (navX.isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(360.0 - navX.getFusedHeading());
		}

		// We have to invert the angle of the NavX so that rotating the robot
		// counter-clockwise
		// makes the angle increase.
		return Rotation2d.fromDegrees(360.0 - toCircle(navX.getYaw()));
	}

	/**
	 * @return SwerveDriveKinematics
	 */
	public SwerveDriveKinematics getDriveKinematics() {
		return driveKinematics;
	}

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
				states[0].angle.getRadians());
		frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[1].angle.getRadians());
		backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[2].angle.getRadians());
		backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
				states[3].angle.getRadians());
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
		//isAngleKept = false;
	}

	public void setKeptAngle(double setKeptAngle) {
		this.setKeptAngle = toCircle(setKeptAngle);
	}

	public void setKeptAngleRelative(double setKeptAngle) {
		if (isBlue) this.setKeptAngle = toCircle(setKeptAngle);
		else this.setKeptAngle = toCircle(setKeptAngle + 180);
	}

	public void setIsAngleKept(boolean isAngleKept) {
		this.isAngleKept = isAngleKept;
	}

	public void setIsBlue(boolean isBlue) {
		this.isBlue = isBlue;
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

	public ProfiledPIDController getXToPositiController() {
		return xToPositioController;
	}

	public ProfiledPIDController getYToPositiController() {
		return yToPositioController;
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

		setIsBlue(Auto.getIsBlue());
		
		double x = chassisSpeeds.vxMetersPerSecond;
		double y = chassisSpeeds.vyMetersPerSecond;

		if ((rotateToAngleController.calculate(getGyroscopeRotation().getDegrees() + 180,
		toCircle(setKeptAngle))) > MAX_REASONABLE_ROTATE_SPEED) {
			ampAngleSpeed = MAX_REASONABLE_ROTATE_SPEED;
		}

		else if ((rotateToAngleController.calculate(getGyroscopeRotation().getDegrees() + 180,
		toCircle(setKeptAngle))) < -MAX_REASONABLE_ROTATE_SPEED) {
			ampAngleSpeed = -MAX_REASONABLE_ROTATE_SPEED;
		}

		else {
			ampAngleSpeed = rotateToAngleController.calculate(getGyroscopeRotation().getDegrees() + 180,
		toCircle(setKeptAngle));
		}

		if(isAngleKept && isBlue) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			y,- x,// -y, x
			ampAngleSpeed, 
			drivetrain.getGyroscopeRotation());
		}

		else if (isAngleKept) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
			-y, x,//y, -x
			ampAngleSpeed,
			drivetrain.getGyroscopeRotation());
		}
		//Untested note lock
		if(noteLock && Vision.getInstance().hasNote() && !Intake.hasNote()){
			canLock = true;
			//NoteX of 0 means we are directly facing the note. Robot relative should then drive it forward towards the note
			chassisSpeeds.omegaRadiansPerSecond = rotateToAngleController.calculate(Vision.getInstance().getNoteXDegrees(), 0);
			
		}else{
			canLock = false;
		}
		SmartDashboard.putBoolean("CanLock", canLock);
		SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(chassisSpeeds);
		setModuleStates(states);
		SmartDashboard.putNumber("Angle", getGyroscopeRotation().getDegrees());
		SmartDashboard.putNumber("ThrottleThing", Control.getThrottle());
		SmartDashboard.putNumber("Other angle", navX.getYaw());
		// SmartDashboard.putNumber("distance thing", backLeftModule.getDriveDistance());
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

    public ChassisSpeeds getChassisSpeeds() {
        return chassisSpeeds;
    }

}

