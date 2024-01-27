package frc.robot.Util;

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
		public final static int FRONT_LEFT_DRIVE = 0;
		public final static int FRONT_LEFT_STEER = 0;
		public final static int FRONT_LEFT_STEER_ENCODER = 0;
		public final static double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(0);

		/* Port and Declaration for Front Right swerve motors & offset */
		public final static int FRONT_RIGHT_DRIVE = 0;
		public final static int FRONT_RIGHT_STEER = 0;
		public final static int FRONT_RIGHT_STEER_ENCODER = 0;
		public final static double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(0);

		/* Port and Declaration for Back left swerve motors & offset */
		public final static int BACK_LEFT_DRIVE = 0;
		public final static int BACK_LEFT_STEER = 0;
		public final static int BACK_LEFT_STEER_ENCODER = 0;
		public final static double BACK_LEFT_STEER_OFFSET = -Math.toRadians(0);

		/* Port and Declaration for Back Right swerve motors & offset */
		public final static int BACK_RIGHT_DRIVE = 0;
		public final static int BACK_RIGHT_STEER = 0;
		public final static int BACK_RIGHT_STEER_ENCODER = 0;
		public final static double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(0);
    }

    public final static class RobotSpecs {
		// The left-to-right distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.75);
		// The front-to-back distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.75);
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

	}

    public final static class Intake{
		public static final int FOLLOW_INTAKE_MOTOR = 0;
		public static final int MASTER_INTAKE_MOTOR = 0;
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
	}

}
