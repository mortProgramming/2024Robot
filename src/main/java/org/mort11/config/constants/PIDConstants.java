package org.mort11.config.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import static org.mort11.config.constants.PhysicalConstants.VOLTAGE;

public class PIDConstants {

    public final static class Arm{
		public static final double POS_KP = VOLTAGE * 0.0066;
		public static final double POS_KI = VOLTAGE * 0;
		public static final double POS_KD = VOLTAGE * 0;
		public static final Constraints POS_CONSTRAINTS = new Constraints(3000, 3000);

        public static final double POS_KG = VOLTAGE * -0.02;
		public static final double POS_KS = VOLTAGE * 0.03;
		public static final double POS_KV = VOLTAGE * 0.00;
		public static final double POS_KA = VOLTAGE * 0;


        public static final double BLOWER_KP = 0.01;
		public static final double BLOWER_KI = 0;
		public static final double BLOWER_KD = 0;

    }

    public final static class Climber {
		public static final double POS_KP = 0.03;
		public static final double POS_KI = 0.00003;
		public static final double POS_KD = 0.0002;
		public static final Constraints POS_CONSTRAINTS = new Constraints(0, 0);
		public static final double POS_POS_TOLERANCE = 1;
    }

    public final static class Drivetrain {
        public final static double TO_POS_KP = 0.5;
		public final static double TO_POS_KI = 0;
		public final static double TO_POS_KD = 0;
		public static final Constraints TO_POS_CONSTRAINTS = new Constraints(10, 10);
		public final static double TO_POS_POS_TOLERANCE = 0.05;

		public final static double TO_ANGLE_KP = 0.07;
		public final static double TO_ANGLE_KI = 0;
		public final static double TO_ANGLE_KD = 0.001;
		public static final Constraints TO_ANGLE_CONSTRAINTS = new Constraints(300, 300);
		public final static double TO_ANGLE_POS_TOLERANCE = 3;
		public final static double TO_ANGLE_VEL_TOLERANCE = 30;


        public static final double AUTON_POS_KP = 0.315;
		public static final double AUTON_POS_KI = 0;
		public static final double AUTON_POS_KD = 0.001;
	
		public static final double AUTON_ROTATION_KP = 1.45;
		public static final double AUTON_ROTATION_KI = 0;
		public static final double AUTON_ROTATION_KD = 0;

		public static final double AUTON_MAX_VELOCITY = 4.17;
    }

    public final static class Wrist{
		public static final double POS_KP = 0.0025;
		public static final double POS_KI = 0;
		public static final double POS_KD = 0;
		public static final Constraints POS_CONSTRAINTS = new Constraints(3000, 3000);

		public static final double POS_KS = 0.00022;
		public static final double POS_GK = 0;
		public static final double POS_KV = 0;
		public static final double POS_KA = 0;
    }
}
