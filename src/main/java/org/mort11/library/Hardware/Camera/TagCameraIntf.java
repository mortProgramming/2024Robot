package org.mort11.library.Hardware.Camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface TagCameraIntf {

    public void setRobotOrientation ( double yaw, double yawRate);

    public boolean hasTag();

    public int getId();

    public Pose2d getRobotPosition();

    public Pose3d get3dRobotPosition();

    public String getCameraName();
    
}
