package org.mort11.config.constants;

public class PortConstants {

    public static final class Controller {
        public static final int JOYSTICK = 0;
        public static final int CONTROLLER = 2;
      }

    public final static class Arm{
		public static final int FOLLOW_ARM_MOTOR = 13;
		public static final int MASTER_ARM_MOTOR = 14;
		public static final int BLOWER_MOTOR = 19;

		public static final int ENCODER_PORT = 9;
    }

    public final static class Climber {
		public static final int FOLLOW_CLIMBER_MOTOR = 24;
		public static final int MASTER_CLIMBER_MOTOR = 15;

		public static final int LEFT_CLIMBER_SERVO = 0;
		public static final int RIGHT_CLIMBER_SERVO = 1;
    }

    public final static class Drivetrain {
		/* Port and Declaration for Front Left swerve motors & offset */
		public final static int FRONT_LEFT_DRIVE_MOTOR = 3;
		public final static int FRONT_LEFT_STEER_MOTOR = 4;
		public final static int FRONT_LEFT_ENCODER = 35;

		/* Port and Declaration for Front Right swerve motors & offset */
		public final static int FRONT_RIGHT_DRIVE_MOTOR = 1;
		public final static int FRONT_RIGHT_STEER_MOTOR = 2;
		public final static int FRONT_RIGHT_ENCODER = 34;

		/* Port and Declaration for Back left swerve motors & offset */
		public final static int BACK_LEFT_DRIVE_MOTOR = 5;
		public final static int BACK_LEFT_STEER_MOTOR = 6;
		public final static int BACK_LEFT_ENCODER = 36; 

		/* Port and Declaration for Back Right swerve motors & offset */
		public final static int BACK_RIGHT_DRIVE_MOTOR = 7;
		public final static int BACK_RIGHT_STEER_MOTOR = 8;
		public final static int BACK_RIGHT_ENCODER = 37;
    }

    public final static class Intake {
		public static final int FOLLOW_INTAKE_MOTOR = 11;
		public static final int MASTER_INTAKE_MOTOR = 12;

		public static final int INTAKE_SENSOR = 0;
    }

    public static final class Lights {
		public static final int LEDS_PORT = 2;
    }

	public static final class Vision {
		public static final String TAG_CAMERA = "taglite";
		public static final String NOTE_CAMERA = "notelite";
    }

    public final static class Wrist{
		public static final int WRIST_MOTOR = 10;

        public static final int TRAP_SERVO_PORT = 8;
    }
}
