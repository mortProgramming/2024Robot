package frc.robot.subsystems;
import frc.robot.Util.Constants.Vision.Pipeline;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class Vision extends SubsystemBase {
    public static Vision vision;

    	private NetworkTable llTable;


    public Vision() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
		return llTable.getEntry("tv").getInteger(0) == 1;
	}

    public double getTargetArea() {
		return llTable.getEntry("ta").getDouble(0);
	}

	public void setPipeline(int id) {
		llTable.getEntry("pipeline").setNumber(id);
	}

	public void setPipeline(Pipeline pipe) {
		setPipeline(pipe.getId());
	}

	public void setPipeline(Pipeline pipe, int ATID) {
		setPipeline(pipe.getId(ATID));
	}

	public Pipeline getPipeline() {
		return Pipeline.getPipeline((int) llTable.getEntry("pipeline").getInteger(0));
	}

	// Translation (x, y, z) Rotation(pitch, yaw, roll)
	public Number[] getCamTran() {
		return llTable.getEntry("targetpose_robotspace").getNumberArray(new Number[0]);
	}

	public double getX() {
		return llTable.getEntry("tx").getDouble(0);
	}

	public double getY() {
		return llTable.getEntry("ty").getDouble(0);
	}

	public double getCamTranX() {
		if (getCamTran().length < 1) {
			return 0;
		}

		return (double) getCamTran()[0];
	}

	public double getCamTranY() {
		if (getCamTran().length < 1) {
			return 0;
		}
		return (double) getCamTran()[1];
	}

	public double getCamTranZ() {
		if (getCamTran().length < 1) {
			return 0;
		}
		return (double) getCamTran()[2];
	}

	public double getCamTranPitch() {
		if (getCamTran().length < 1) {
			return 0;
		}
		return (double) getCamTran()[3];
	}

	public double getCamTranYaw() {
		if (getCamTran().length < 1) {
			return 0;
		}
		return (double) getCamTran()[4];
	}

	public double getCamTranRoll() {
		if (getCamTran().length < 1) {
			return 0;
		}
		return (double) getCamTran()[5];
	}

	public Pose2d getPose() {
		double[] poseNum = new double[6];

		if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
			poseNum = llTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
		} else {
			poseNum = llTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
		}

		return new Pose2d(poseNum[0], poseNum[1], new Rotation2d(Math.toRadians(poseNum[5])));

	}

	public double getLatency() {
		return llTable.getEntry("botpose").getDoubleArray(new double[6])[6];
	}

	public int getATId() {
		return (int) llTable.getEntry("tid").getInteger(0);
	}

	public static Vision getInstance() {
		if (vision == null) {
			vision = new Vision();
		}
		return vision;
	}
}
