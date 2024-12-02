package org.mort11.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.mort11.config.constants.PortConstants.Intake.*;

import static org.mort11.mortlib.hardware.motor.MotorTypeEnum.*;
import static org.mort11.mortlib.logger.LoggerTypeEnum.*;
import org.mort11.mortlib.hardware.motor.MotorGroup;
import org.mort11.mortlib.logger.LoggerGroup;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake intake;

    //left is main motor
    private MotorGroup motors;

    private LoggerGroup logger;

    private static DigitalInput input = new DigitalInput(INTAKE_SENSOR);

    private double intakeSpeed;

    private Intake() {
        motors = new MotorGroup(FALCON, MASTER_INTAKE_MOTOR, FOLLOW_INTAKE_MOTOR);
        motors.setDirectionFlip(1, true);

        input = new DigitalInput(INTAKE_SENSOR);

        intakeSpeed = 0;

        logger = new LoggerGroup("Intake", SMARTDASHBOARD, SHUFFLEBOARD);
        logger.putBoolean("Piece In", this::hasNote);
    }

    @Override
    public void periodic() {
        motors.setVoltage(intakeSpeed);
    }

    public void setIntakeVelocity(double intakeSpeed){
        this.intakeSpeed = intakeSpeed;
    }

    public boolean hasNote() {
        return !input.get();
    }

    public static boolean hasNoteStatic() {
        return !input.get();
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }
}
