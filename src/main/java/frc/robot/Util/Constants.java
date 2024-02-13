package frc.robot.util;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public final static class PeripheralPorts {
		public final static int JOYSTICK = 0;	// Joystick port & Declaration
		public final static int THROTTLE = 1;	// Throttle port & Declaration
		public final static int XBOX_CONTROLLER = 2;	// Xbox Controller & Declaration
	}

    public final static class Drivetrain {
		/* Port and Declaration for Front Left swerve motors & offset */
		public final static int FRONT_LEFT_DRIVE = 3;
		public final static int FRONT_LEFT_STEER = 4;
		public final static int FRONT_LEFT_STEER_ENCODER = 35;
		public final static double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(144.1);

		/* Port and Declaration for Front Right swerve motors & offset */
		public final static int FRONT_RIGHT_DRIVE = 1;
		public final static int FRONT_RIGHT_STEER = 2;
		public final static int FRONT_RIGHT_STEER_ENCODER = 34;
		public final static double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(269.0);

		/* Port and Declaration for Back left swerve motors & offset */
		public final static int BACK_LEFT_DRIVE = 5;
		public final static int BACK_LEFT_STEER = 6;
		public final static int BACK_LEFT_STEER_ENCODER = 36;
		public final static double BACK_LEFT_STEER_OFFSET = -Math.toRadians(115.7);

		/* Port and Declaration for Back Right swerve motors & offset */
		public final static int BACK_RIGHT_DRIVE = 7;
		public final static int BACK_RIGHT_STEER = 8;
		public final static int BACK_RIGHT_STEER_ENCODER = 37;
		public final static double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(39.0);

		public final static double XVALUE_KP = 1.1;
		public final static double XVALUE_KI = 0;
		public final static double XVALUE_KD = 0;
		public final static double XVALUE_TOLERANCE = 0.01;

		public final static double YVALUE_KP = 1.1;
		public final static double YVALUE_KI = 0;
		public final static double YVALUE_KD = 0;
		public final static double YVALUE_TOLERANCE = 0.05;

		public final static double OMEGAVALUE_KP = 0.02;
		public final static double OMEGAVALUE_KI = 0;
		public final static double OMEGAVALUE_KD = 0;
		public final static double OMEGAVALUE_TOLERANCE = 0;
    }

    public final static class RobotSpecs {
		// The left-to-right distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19);
		// The front-to-back distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19);
		//	Maximum voltage that will be used by subsystem it is called to
		public static final double MAX_VOLTAGE = 12.0;	

		//	The maximum velocity that can be travelled by swerve system (radians per second and meters per second)
		public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0
				* SdsModuleConfigurations.MK4I_L2.getDriveReduction()
				* SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI) * 0.99; // 100% ~4.97 m/s
		public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
				/ Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
		
		//	Maximum velocity and acceleration during auton period
		public static final double MAX_VELOCITY_AUTO = 4.3;
		public static final double MAX_ACCELERATION_AUTO = 4;

		//Max and minimum throttle ports & declarations
        public static final double MAX_THROTTLE = 0;
        public static final double MIN_THROTTLE = 0;
        public static final double DEAD_BAND = 0;
	}

	public final static class Climber{
		public static final int FOLLOW_CLIMBER_MOTOR = 16;
		public static final int MASTER_CLIMBER_MOTOR = 15;
		public static final double POSITION_PID_P = 0;
		public static final double POSITION_PID_I = 0;
		public static final double POSITION_PID_D = 0;
		public static final double POSITION_PID_V = 0;
		public static final double POSITION_PID_A = 0;

		public static final double POSITION_FF_S = 0;
		public static final double POSITION_FF_G = 0;
		public static final double POSITION_FF_V = 0;
		public static final double POSITION_FF_A = 0;

		public static final double CLIMBER_REST_POSITION = 0;
		public static final double CLIMBER_RETRACT_POSITION = 0;
		public static final double CLIMBER_MAX_POSITION = 0;

		public static final double CLIMBER_NEAR_SETPOINT_ERROR = 0;
	}

    public final static class Intake{
		public static final int FOLLOW_INTAKE_MOTOR = 11;
		public static final int MASTER_INTAKE_MOTOR = 12;

		public static final double INTAKE_SPEED = 0;

		public static final int INTAKE_SENSOR = 0;
	}
	
	public final static class Wrist{
		public static final int WRIST_MOTOR = 10;

		public static final double POSITION_PID_P = 0;
		public static final double POSITION_PID_I = 0;
		public static final double POSITION_PID_D = 0;
		public static final double POSITION_PID_V = 0;
		public static final double POSITION_PID_A = 0;

		public static final double POSITION_FF_S = 0;
		public static final double POSITION_FF_G = 0;
		public static final double POSITION_FF_V = 0;
		public static final double POSITION_FF_A = 0;

		public static final double WRIST_REST_POSITION = 0.2;
		public static final double WRIST_SCORE_POSITION = 1.3;
		public static final double WRIST_INTAKE_POSITION = 7.9;
		public static final double WRIST_TRAP_POSITION = 0;

		public static final double WRIST_NEAR_SETPOINT_ERROR = 0;

		public static final double WRIST_SPEED = 0;
	}

	public final static class Arm{
		public static final int FOLLOW_ARM_MOTOR = 13;
		public static final int MASTER_ARM_MOTOR = 14;

		public static final double POSITION_PID_P = 0.5;
		public static final double POSITION_PID_I = 0;
		public static final double POSITION_PID_D = 0;
		public static final double POSITION_PID_V = 0;
		public static final double POSITION_PID_A = 0;


		// simple feed forward plus guess
		// public static final double POSITION_FF_S = 0.52771;
		// public static final double POSITION_FF_G = 0.04;
		// public static final double POSITION_FF_V = 13.199;
		// public static final double POSITION_FF_A = 0.99537;

		public static final double POSITION_FF_S = 0.52771;
		public static final double POSITION_FF_G = 0.38738;
		public static final double POSITION_FF_V = 14.112;
		public static final double POSITION_FF_A = 0.81092;

		public static final double ARM_REST_POSITION = 0.7;
		// public static final double ARM_AMP_POSITION = -25;
		public static final double ARM_AMP_POSITION = -90;
		public static final double ARM_INTAKE_POSITION = -2.5;
		public static final double ARM_TRAP_POSITION = 0;
		public static final double ARM_SPEAKER_POSITION = 0;
		
		public static final double ARM_NEAR_SETPOINT_ERROR = 5;

		public static final double ARM_DEGREES_TO_0 = -12;
		public static final double ARM_GEAR_RATIO = 0.25;
	}

	public final static class Vision {
		public static enum Pipeline {
			DEFAULT(0), APRIL_TAG(1), TAPE(9);

			int id;

			Pipeline(int id) {
				this.id = id;
			}

			public int getId() {
				return id;
			}

			/**
			 * Gets id of pipeline for a specific April Tag ID, id 1 at pipeline 3, etc.
			 */
			public int getId(int ATID) {
				if (this.id == 1) {
					return ATID;
				}
				return id;
			}

			public static Pipeline getPipeline(int id) {
				for (Pipeline p : values()) {
					if (p.getId() == id) {
						return p;
					}
				}
				return DEFAULT;
			}
		}

		//TODO find camera height
		public static final double CAMERA_HEIGHT = 0;

		public static final int AMOUNT_TEST_FRAMES = 0;
		public static final int MAX_OUTLIERS = 0;
		public static final int MAX_NON_OUTLIERS = 0;
		public static final double MAX_ERROR = 0;

		public static final double CAMERA_MOUNT_ANGLE = 0;

		public static final double MAX_POSE_ERROR_METERS = 1;
	}

	public static final class AprilTagData{

		public static final class Red{

			public static final class Amp{
						
				public static final int ID = 5;
				public static final double height = 48.125;
					
			}
			public static final class Speaker{
				public static final int IDCenter = 4;
				public static final int IDOffset = 3;
				public static final double height = 48.625;
			}
			public static final class Source{
				public static final int IDfar = 9;
				public static final int IDclose = 10;
				public static final double height = 48.125;
			}
			public static final class Stage{
				public static final int IDclose = 13;
				public static final int IDleftClose = 11;
				public static final int IDrightClose = 12;
				public static final double height = 47.5;
			}		
		}
		public static class Blue{

			public static final class Amp{

				public static final int ID = 6;
				public static final double height = 48.125;
			}
			public static final class Speaker{
				public static final int IDCenter = 7;
				public static final int IDOffset = 8;
				public static final double centerHeight = 48.625;
				public static final double offsetHeight = 48.625;
			}
			public static final class Source{
				public static final int IDfar = 2;
				public static final int IDclose = 1;
				public static final double height = 48.125;
			}
			public static final class Stage{
				public static final int IDfar = 14;
				public static final int IDleftClose = 15;
				public static final int IDrightClose = 16;
				public static final double height = 47.5;
			}
		}
	}

}