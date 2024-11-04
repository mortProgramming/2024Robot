package org.mort11.library.Swerve;

import static org.mort11.library.Swerve.Constants.*;

import org.mort11.library.Hardware.Brands.CTRE.CTREUtility.Falcon500;
import org.mort11.library.Hardware.Brands.CTRE.CTREUtility.Krakenx60;
import org.mort11.library.Hardware.Brands.REV.RevUtility.NEO;
import org.mort11.library.Hardware.Brands.REV.RevUtility.NEO550;
import org.mort11.library.Hardware.Encoder.Encoder;
import org.mort11.library.Hardware.Encoder.EncoderTypeEnum;
import org.mort11.library.Hardware.Motor.Motor;
import org.mort11.library.Hardware.Motor.MotorTypeEnum;
import org.mort11.library.Hardware.Motor.PIDMotor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    public int driveMotorID;
    public int steerMotorID;
    public int encoderID;

    public MotorTypeEnum driveMotorType;
    public MotorTypeEnum steerMotorType;
    public EncoderTypeEnum encoderType;
    public ModuleTypeEnum moduleType;

    public Motor driveMotor;
    public PIDMotor steerMotor;
    public Encoder encoder;

    public double maxSpeed;
    public double maxVoltage;
    public double rotationToMeters;

    public double offset;

    public SwerveModuleState state;

    public SwerveModule(MotorTypeEnum driveMotorType, int driveMotorID, 
            MotorTypeEnum steerMotorType, int steerMotorID, 
            EncoderTypeEnum encoderType, int encoderID,
            ModuleTypeEnum moduleType
        ) {
        this.driveMotorID = driveMotorID;
        this.driveMotorID = driveMotorID;
        this.encoderID = encoderID;

        this.driveMotorType = driveMotorType;
        this.steerMotorType = steerMotorType;
        this.encoderType = encoderType;
        this.moduleType = moduleType;

        driveMotor = new Motor(driveMotorType, driveMotorID);
        steerMotor = new PIDMotor(steerMotorType, steerMotorID);
        encoder = new Encoder(encoderType, encoderID);

        maxSpeed = Math.PI / 60;
        maxVoltage = 12;
        offset = 0;
        rotationToMeters = Math.PI;

        switch(driveMotorType) {
            case NEO:
                maxSpeed = maxSpeed * NEO.MAX_RPM;
                break;
            case NEO550:
                maxSpeed = maxSpeed * NEO550.MAX_RPM;
                break;
            case FALCON:
                maxSpeed = maxSpeed * Falcon500.MAX_RPM;
                break;
            case KRAKEN:
                maxSpeed = maxSpeed * Krakenx60.MAX_RPM;
                break;
        }

        switch (steerMotorType) {
            case NEO:
                steerMotor.setPIDValues(
                    NEO.SWERVE_STEER_KP, 
                    NEO.SWERVE_STEER_KI, 
                    NEO.SWERVE_STEER_KD
                );
                break;
            case NEO550:
                steerMotor.setPIDValues(
                    NEO550.SWERVE_STEER_KP, 
                    NEO550.SWERVE_STEER_KI, 
                    NEO550.SWERVE_STEER_KD
                );
                break;
            case FALCON:
                steerMotor.setPIDValues(
                    Falcon500.SWERVE_STEER_KP, 
                    Falcon500.SWERVE_STEER_KI, 
                    Falcon500.SWERVE_STEER_KD
                );
                break;
            case KRAKEN:
                steerMotor.setPIDValues(
                    Krakenx60.SWERVE_STEER_KP, 
                    Krakenx60.SWERVE_STEER_KI, 
                    Krakenx60.SWERVE_STEER_KD
                );
                break;
        }

        steerMotor.setPIDEnableContinuousInput(0, 1);
        steerMotor.setPIDTolerance(0.001, 10000);

        switch (moduleType) {
            case MK4i:
                maxSpeed = maxSpeed * MK4i.WHEEL_DIAMETER * MK4i.DRIVE_REDUCTION;
                rotationToMeters = rotationToMeters * MK4i.WHEEL_DIAMETER * MK4i.DRIVE_REDUCTION;
        }
    }

    public void setCurrentLimits(double limit) {
        driveMotor.setCurrentLimit(limit);
        steerMotor.setCurrentLimit(limit);
    }

    public void setPosition(Rotation2d setpoint) {
        steerMotor.setPositionRotations(getEncoderPosition().getRotations(), setpoint.getRotations());
    }

    public void setDrivePercent(double percent) {
        driveMotor.setPercent(percent);
    }

    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    public void setDriveSpeedMeters(double speedMeters) {
        driveMotor.setVoltage((speedMeters / maxSpeed) * maxVoltage);
    }

    public void setModuleState(SwerveModuleState state) {
        this.state = state;

        SwerveModuleState.optimize(state, getEncoderPosition());
        setDriveSpeedMeters(state.speedMetersPerSecond);
        setPosition(state.angle);
    }

    public void setMaxVoltage(double maxVoltage) {
        this.maxVoltage = maxVoltage;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }



    public Rotation2d getEncoderPosition() {
        return Rotation2d.fromDegrees(encoder.getPosition().getDegrees() - offset);
    }

    public double getDrivePositionRotations() {
        return driveMotor.getPositionRotations();
    }

    public double getDriveVelocityRPM() {
        return driveMotor.getVelocityRPM();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (driveMotor.getPositionRotations() * rotationToMeters), 
            getEncoderPosition()
        );
    }



    public MotorTypeEnum getDriveMotorType() {
        return driveMotorType;
    }
    
    public MotorTypeEnum getSteerMotorType() {
        return steerMotorType;
    }

    public EncoderTypeEnum getEncoderType() {
        return encoderType;
    }

    public ModuleTypeEnum getModuleType() {
        return moduleType;
    }

}