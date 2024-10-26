package org.mort11.configuration;

public final class Constants {

	public final static class Climber{

		public static final int CLIMBER_LOCK_ANGLE = 25;

		public static final double LEFT_CLIMBER_REST_POSITION = 177;
		public static final double LEFT_CLIMBER_MAX_POSITION = 34;

		public static final double RIGHT_CLIMBER_MAX_POSITION = -32;
		public static final double RIGHT_CLIMBER_REST_POSITION = -173;

		public static final double CLIMBER_NEAR_SETPOINT_ERROR = 0;

		public static final double CLIMBER_UP_SPEED = 0.5;
		public static final double CLIMBER_DOWN_SPEED = -0.5;

		public static final double SERVO_GLOBAL_LOCK_POSITION = 90;
		public static final double RIGHT_UNLOCK_POSITION = 45;
		public static final double LEFT_UNLOCK_POSITION = 135;

	}

    public final static class Intake{

		public static final double INTAKE_SPEED = 0.6;
		public static final double AMP_SHOOT_SPEED = -0.3;
		public static final double SHOOTER_SHOOT_SPEED = -.80;
		public static final double AUTO_SHOOT_SPEED = -0.45;
	}
	
	public final static class Wrist{

		//rotation for positions
		// public static final double WRIST_REST_POSITION = 0.2;
		// public static final double WRIST_SCORE_POSITION = 1.3;
		// public static final double WRIST_INTAKE_POSITION = 7.9;
		// public static final double WRIST_TRAP_POSITION = 0;
		public static final double WRIST_REST_POSITION = -22;
		public static final double WRIST_SCORE_POSITION = 0;
		public static final double WRIST_INTAKE_POSITION = 200;
		public static final double WRIST_TRAP_POSITION = 141;
		public static final double WRIST_SPIT_POSITION = 41;
		public static final double WRIST_FLOORTRAP_POSITION = 151;

		public static final double WRIST_NEAR_SETPOINT_ERROR = 0;

		public static final double WRIST_SPEED = 0;

		public static final double WRIST_DEGREES_TO_0 = 0;
		public static final double WRIST_GEAR_RATIO = -25;


		public static final double TRAP_SERVO_REST_POS = 80;
		public static final double TRAP_SERVO_POS = -11.5;
	}

	public static final class Lights {
		public static final int LEDS_PORT = 2;

		public static final double GREEN_COLOR = 0.77;
		public static final double RED_COLOR = 0.61; //0.61
		public static final double BLUE_COLOR = 0.87;
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