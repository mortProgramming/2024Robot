package org.mort11.library.Arm;

import org.mort11.library.Hardware.Motor.Motor;
import org.mort11.library.Hardware.Motor.MotorIntf;
import org.mort11.library.Hardware.Motor.MotorTypeEnum;

import edu.wpi.first.math.geometry.Rotation2d;

public class HeldArm {
    
    public MotorIntf motor;

    public int kG;

    public Rotation2d offset;

    public HeldArm(MotorTypeEnum motorType, int motorID) {
        motor = new Motor(motorType, motorID);
    }

    public HeldArm(MotorIntf motor) {
        this.motor = motor;
    }

    public void setG(int kG, Rotation2d offset) {
        this.kG = kG;
        this.offset = offset;
    }

    public void setG(int kG, double offset) {
        this.kG = kG;
        this.offset = Rotation2d.fromRotations(offset);
    }

    public void setVoltage (double voltage) {
        motor.setVoltage(voltage);
    }

    public void setHeldVoltage (double voltage, Rotation2d position) {
        motor.setVoltage(voltage + kG * position.rotateBy(offset).getSin());
    }

    public void setHeldVoltage (double voltage, double position) {
        motor.setVoltage(voltage + kG * Rotation2d.fromRotations(position).rotateBy(offset).getSin());
    }
}
