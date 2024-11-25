package org.mort11.mortlib.arm;

import org.mort11.mortlib.hardware.motor.Motor;
import org.mort11.mortlib.hardware.motor.MotorIntf;
import org.mort11.mortlib.hardware.motor.MotorTypeEnum;

import edu.wpi.first.math.geometry.Rotation2d;

public class HeldArm {
    public MotorIntf motor;

    public double kG;

    public Rotation2d offset;

    public HeldArm(MotorTypeEnum motorType, int motorID) {
        motor = new Motor(motorType, motorID);

        kG = 0;
        offset = Rotation2d.fromDegrees(0);
    }

    public HeldArm(MotorIntf motor) {
        this.motor = motor;

        kG = 0;
        offset = Rotation2d.fromDegrees(0);
    }

    public void setG(double kG) {
        this.kG = kG;
    }

    public void setG(double kG, Rotation2d offset) {
        this.kG = kG;
        this.offset = offset;
    }

    public void setG(double kG, double offset) {
        this.kG = kG;
        this.offset = Rotation2d.fromRotations(offset);
    }

    public void setVoltage (double voltage) {
        motor.setVoltage(voltage);
    }

    public void setHeldVoltage(double voltage, Rotation2d position) {
        motor.setVoltage(voltage + kG * position.rotateBy(offset).getCos());
    }

    public void setHeldVoltage(double voltage, double position) {
        motor.setVoltage(voltage + kG * Rotation2d.fromRotations(position).rotateBy(offset).getCos());
    }
}
