package org.mort11.configuration.constants;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;

public class PhysicalConstants {

    public static final class Controller {
        public static final int JOYSTICK_X_CHANNEL = 0;
        public static final int JOYSTICK_Y_CHANNEL = 1;
        public static final int JOYSTICK_TWIST_CHANNEL = 3;
        public static final int THROTTLE_CHANNEL = 2;
    
        public static final double LATERAL_DEAD_BAND = 0.025;
        public static final double ROTATE_DEAD_BAND = 0.1;
    
        public static final double MAX_LATERAL_THROTTLE = 1;
        public static final double MIN_LATERAL_THROTTLE = 0.3;

        public static final double MAX_ROTATE_THROTTLE = 0.4;
        public static final double MIN_ROTATE_THROTTLE = 0.2;
    }

    public final static class Arm{
		public static final double ARM_REST_POSITION = -32;
		public static final double ARM_AMP_POSITION = 92;  //169
		public static final double ARM_INTAKE_POSITION = -25;
		public static final double ARM_TRAP_POSITION = 38;
		public static final double ARM_PRETRAP_POSITION = 130;
		public static final double ARM_FLOORTRAP_POSITION = 50;

		public static final double ARM_ENCODER_DEGREES_TO_0 = 84;
		public static final double ARM_NEVER_POSITION = 90;

		public static final double ARM_WRIST_TIMEOUT = 1;

		public static final double BLOWER_MOTOR_MAX_SPEED = 1;
    }

    public final static class Climber {
		
    }

    public static final class Drivetrain {
        // The left-to-right distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(19);
		// The front-to-back distance between the drivetrain wheels measured from center
		// to center.
		public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19);

        public static final double DRIVEBASE_RADIUS_METERS = Math.hypot((DRIVETRAIN_TRACKWIDTH_METERS / 2), (DRIVETRAIN_WHEELBASE_METERS / 2));

        public static final double MAX_VELOCITY_METERS_PER_SECOND = (6000 / 60.0
				* SdsModuleConfigurations.MK4I_L3.getDriveReduction()
				* SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI) * 0.99; // 100% ~4.97 m/s

		public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
				/ Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final double IMU_TO_ROBOT_FRONT_ANGLE = 0;

        public static final double FRONT_LEFT_OFFSET = -Math.toRadians(197);
		public static final double FRONT_RIGHT_OFFSET = -Math.toRadians(200);
		public static final double BACK_LEFT_OFFSET = -Math.toRadians(100);
		public static final double BACK_RIGHT_OFFSET = -Math.toRadians(233);
    }

    public final static class Intake {
		
    }

    public static final class Lights {
		
    }

    public final static class Wrist{
        
    }
}
