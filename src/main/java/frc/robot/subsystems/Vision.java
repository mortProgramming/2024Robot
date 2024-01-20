package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static Vision vision;

    	private NetworkTable llTable;


    public Vision() {
        llTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * 
     * @return 
     */
    public static Vision getInstance() {
		if (vision == null) {
			vision = new Vision();
		}
		return vision;
	}
}
