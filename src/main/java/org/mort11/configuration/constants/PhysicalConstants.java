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

        public static final double MAX_ROTATE_THROTTLE = 0.3;
        public static final double MIN_ROTATE_THROTTLE = 0.1;
    }

    public final static class Arm{
		public static final double ARM_REST_POS = -32;
		public static final double ARM_AMP_POS = 92;  //169
		public static final double ARM_TRAP_POS = 38;
		public static final double ARM_PRETRAP_POS = 130;
		public static final double ARM_FLOORTRAP_POS = 50;

		public static final double ARM_ENCODER_TO_0_DEGREES = 84;
		public static final double ARM_NEVER_POS = 90;

		public static final double ARM_WRIST_TIMEOUT = 1;

		public static final double BLOWER_MOTOR_MAX_SPEED = -1;
    }

    public final static class Climber {
      public static final double LEFT_CLIMBER_REST_POS = 177;
		  public static final double LEFT_CLIMBER_MAX_POS = 34;

      public static final double RIGHT_CLIMBER_REST_POS = -173;
		  public static final double RIGHT_CLIMBER_MAX_POS = -32;

      public static final double MANUAL_CLIMBER_SPEED = 1;

		  public static final double SERVO_GLOBAL_LOCK_POS = 90;
		  public static final double RIGHT_UNLOCK_POS = 45;
		  public static final double LEFT_UNLOCK_POS = 135;
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

    public static final double IMU_TO_ROBOT_FRONT_ANGLE = 90;

    public static final double FRONT_LEFT_OFFSET = -Math.toRadians(197);
		public static final double FRONT_RIGHT_OFFSET = -Math.toRadians(200);
		public static final double BACK_LEFT_OFFSET = -Math.toRadians(100);
		public static final double BACK_RIGHT_OFFSET = -Math.toRadians(233);

    }

    public final static class Intake {
      public static final double INTAKE_SPEED = 0.6;
		  public static final double AMP_SHOOT_SPEED = -0.3;
		  public static final double SHOOTER_SHOOT_SPEED = -0.80;
		  public static final double AUTO_SHOOT_SPEED = -0.45;

      public static final double SENSOR_MIN_TIME = 0.05;
    }

    public static final class Lights {
      public static final double RED_COLOR = 0.61;
      public static final double GREEN_COLOR = 0.77;
		  public static final double BLUE_COLOR = 0.87;
    }

    public final static class Wrist {
      public static final double WRIST_REST_POS = -22;
      public static final double WRIST_INTAKE_POS = 200;
      public static final double WRIST_FLOORTRAP_POS = 151;
      public static final double WRIST_TRAP_POS = 141;
      public static final double WRIST_SPIT_POS = 41;
  
      public static final double WRIST_DEGREES_TO_0 = 0;
      public static final double WRIST_GEAR_RATIO = -25;

      public static final double TRAP_SERVO_REST_POS = 80;
      public static final double TRAP_SERVO_POS = 0;
    }

    public final static class Vision {
      public static final double MAX_POSE_ERROR_METERS = 1;
    } 
}
