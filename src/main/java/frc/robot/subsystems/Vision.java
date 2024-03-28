package frc.robot.subsystems;
import static frc.robot.utility.Constants.Vision.AMOUNT_TEST_FRAMES;
import static frc.robot.utility.Constants.Vision.MAX_OUTLIERS;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {
    public static Vision vision;
		private Alliance defaultAlliance = Alliance.Blue;
    	private NetworkTable tagTable;
		private NetworkTable intakeTable;
	
		
		private double[] values = new double[AMOUNT_TEST_FRAMES];
		private double[] outliers = new double[MAX_OUTLIERS];
		private VideoSource intakeCam;
		
		
    public Vision() {
        // tagTable = NetworkTableInstance.getDefault().getTable("limelight");
		tagTable = NetworkTableInstance.getDefault().getTable("limelight-tag");
		intakeTable = NetworkTableInstance.getDefault().getTable("limelight-intake");
    }

	/**
	 * 
	 * @return True if the limelight has a target, false if no target
	 */
    public boolean hasTag() {
		return tagTable.getEntry("tv").getDouble(0) == 1.0;
	}
	/**
	 * 
	 * @return True if the limelight has a target, false if no target
	 */
    public boolean hasNote() {
		return intakeTable.getEntry("tv").getDouble(0) == 1.0;
	}

	public double getNoteXDegrees() {
		return intakeTable.getEntry("tx").getDouble(0);
	}

	/**
	 * 
	 * @return The area of the targeted note piece, if any. -1 if no target found
	 */
	public double getNoteArea() {
		if(hasNote()){
			return intakeTable.getEntry("ta").getDouble(0);
		}
		return -1;
		
	}

	public void setNoteCamLights(int input) {
		intakeTable.getEntry("ledMode").setNumber(input);
	}

	/**
	 * 
	 * @return The area of the targeted apriltag, if any. -1 if no target found
	 */
    public double getTagArea() {
		if(hasTag()){
			return tagTable.getEntry("ta").getDouble(0);
		}
		return -1;
		
	}

	// Translation (x, y, z) Rotation(pitch, yaw, roll)
	/**
	 * 
	 * @return
	 */
	public Number[] getCamTranslation() {
		return tagTable.getEntry("targetpose_robotspace").getNumberArray(new Number[0]);
	}

	/**
	 * Get the X value of the target in degrees
	 * @return The X value of the target in degrees, -1 if no target found.
	 */
	public double getX() {
		if(hasTag()){
			return tagTable.getEntry("tx").getDouble(0);
		}
		return -1;
	}

	/**
	 * Get the Y value of the target in degrees
	 * @return The Y value of the target in degrees, -1 if no target found
	 */
	public double getY() {
		if(hasTag()){
			return tagTable.getEntry("ty").getDouble(0);
		}
		return -1;
	}

	public double getCamTranslationX() {
		if (getCamTranslation().length < 1) {
			return 0;
		}

		return (double) getCamTranslation()[0];
	}

	public double getCamTranslationY() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[1];
	}

	public double getCamTranslationZ() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[2];
	}


	public double getCamTranslationPitch() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[3];
	}

	
	public double getCamTranslationYaw() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[4];
	}

	
	public double getCamTranslationRoll() {
		if (getCamTranslation().length < 1) {
			return 0;
		}
		return (double) getCamTranslation()[5];
	}

	/**
	 * 
	 * @return
	 */
	public Pose2d getPose() {
		double[] poseNum = new double[6];
		
		poseNum = tagTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}

	/**
	 * 
	 * @return
	 */
	public double getLatency() {
		return tagTable.getEntry("botpose").getDoubleArray(new double[6])[6];
	}

	/**
	 * Get the ID of any currently targeted AprilTag
	 * @return ID of targeted apriltag as an int, -1 if no target found
	 */
	public int getAprilTagId() {
		if(hasTag()){
			return (int) tagTable.getEntry("tid").getInteger(0);
		}
		return -1;
		
	}


	// old, prior to 3d vision
	/**
	 * Gets the distance to the target AprilTag in inches
	 * @param tagHeight 
	 * The Height of the tag. All tag heights are in constants
	 * @return The Height of the tag in inches. If no target, return -1
	 */
	// public double getDistanceToTag(double tagHeight) {
	// 	if(tagTable.getEntry("tv").getDouble(0) == 1){
	// 		return (tagHeight - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(getY())));
	// 	}
	// 	return -1;
		
	// }
	/*
	 * Gets the robots pose on the field relative to the driverstation.
	 */
	public Pose2d getFieldPose() {
		double[] poseNum = new double[6];
		poseNum = tagTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));
	}
	
	public double[] getFieldPoseAsArray() {
		return tagTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
	}

	/**
     * 
     */
    @Override
    public void periodic() {
		
		SmartDashboard.putNumber("note x", getNoteXDegrees());
		SmartDashboard.putNumberArray("Field Positioning", getFieldPoseAsArray());
		SmartDashboard.putBoolean("HASTAG", hasTag());
		
    }

	/**
	 * 
	 */
	
	public static Vision getInstance() {
		if (vision == null) {
			vision = new Vision();
		}
		return vision;
	}
}
