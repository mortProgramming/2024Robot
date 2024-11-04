package org.mort11.library.Hardware.IMU;

import org.mort11.library.Hardware.Brands.CTRE.Pigeon2IMU;
import org.mort11.library.Hardware.Brands.KauaiLabs.NavX2IMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class IMU implements IMUIntf {

    public IMUTypeEnum imuType;
    public int ID;

    public IMUIntf imu;

    public IMU(IMUTypeEnum imuType, int ID) {
        this.imuType = imuType;
        this.ID = ID;

        switch (imuType) {
            case PIGEON2:
                imu = new Pigeon2IMU(ID);
                break;
            case NAVX2:
                imu = new NavX2IMU(ID);
                break;
        }
    }

    public void setCanivore(String canivore) {
        imu.setCanivore(canivore);
    }

    public double getAngle() {
        return imu.getAngle();
    }

    public double getRate() {
        return imu.getRate();
    }

    public void reset () {
        imu.reset();
    }
    
    public Rotation2d getRotation2d() {
        return imu.getRotation2d();
    }

    public Rotation3d getRotation3d() {
        return imu.getRotation3d();
    }

    public IMUTypeEnum getIMUType() {
        return imuType;
    }

}