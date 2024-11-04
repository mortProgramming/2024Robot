package org.mort11.library.Arm;

import org.mort11.library.Hardware.Motor.MotorIntf;
import org.mort11.library.Hardware.Motor.MotorTypeEnum;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class PIDArm extends HeldArm {

    public ProfiledPIDController controller;

    public PIDArm(MotorTypeEnum motorType, int motorID) {
        super(motorType, motorID);

        controller = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
    }
    
    public PIDArm (MotorIntf motor) {
        super(motor);
    }

    public void setPIDConstants (double kP, double kI, double kD, double kV, double kA) {
        controller = new ProfiledPIDController(kP, kI, kD, new Constraints(kV, kA));
    }

    public void setPIDEnableContinuousInput(Rotation2d minimum, Rotation2d maximum) {
        controller.enableContinuousInput(minimum.getRotations(), maximum.getRotations());
    }

    public void setPIDEnableContinuousInput(double minimum, double maximum) {
        controller.enableContinuousInput(minimum, maximum);
    }

    public void setPIDDisableContinuousInput() {
        controller.disableContinuousInput();
    }

    public void setPIDTolerance(Rotation2d position, Rotation2d velocity) {
        controller.setTolerance(position.getRotations(), velocity.getRotations());
    }

    public void setPIDTolerance(double position, double velocity) {
        controller.setTolerance(position, velocity);
    }

    public void setPIDPosition (Rotation2d currentPosition, Rotation2d wantedPosition) {
        setHeldVoltage(controller.calculate(currentPosition.getRotations(), wantedPosition.getRotations()), currentPosition);
    }

    public void setPIDPosition (double currentPosition, double wantedPosition) {
        setHeldVoltage(controller.calculate(currentPosition, wantedPosition), currentPosition);
    }
}
