package org.mort11.subsystems;

import static org.mort11.config.constants.PIDConstants.Arm.*;
import static org.mort11.config.constants.PhysicalConstants.Arm.*;
import static org.mort11.config.constants.PortConstants.Arm.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
    private static Arm arm;

    //left is main motor
    private TalonFX masterArmMotor;
    private TalonFX followArmMotor;
    private CANSparkMax blowerMotor;

    private double armSpeed;

    private double targetBlowerOutput;
    private double currentBlowerOutput;

    private ProfiledPIDController armPositionController;
    private PIDController blowerController;

    private DutyCycleEncoder encoder;

    public Arm() {
        masterArmMotor = new TalonFX(MASTER_ARM_MOTOR);
        followArmMotor = new TalonFX(FOLLOW_ARM_MOTOR);
        encoder = new DutyCycleEncoder(ENCODER_PORT);
        blowerMotor = new CANSparkMax(BLOWER_MOTOR, MotorType.kBrushless);

        followArmMotor.setControl(new Follower(MASTER_ARM_MOTOR, true));

        targetBlowerOutput = 0;
        currentBlowerOutput = 0;

        armPositionController = new ProfiledPIDController(
            POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS
        );

        blowerController = new PIDController(BLOWER_KP, BLOWER_KI, BLOWER_KD);
    }

    @Override
    public void periodic() {
        masterArmMotor.set(armSpeed);

        currentBlowerOutput += blowerController.calculate(currentBlowerOutput,targetBlowerOutput);
        blowerMotor.set(currentBlowerOutput);

        SmartDashboard.putNumber("Encoder Arm Pos Degrees", getEncoderPosDeg());
        SmartDashboard.putNumber("arm output", armSpeed);
        SmartDashboard.putNumber("ActualArmMotorOutput", masterArmMotor.get());
        SmartDashboard.putNumber("Blower Value", currentBlowerOutput);
    }

    public void setArmVelocity(double armSpeed){
        this.armSpeed = armSpeed + getGravityOffset();
    }

    public void setSetpoint(double setpoint){
        this.armSpeed = -armPositionController.calculate(getEncoderPosDeg(), setpoint) +
            getGravityOffset();
    }

    public void setBlowerTarget(double targetBlowerOutput){
        this.targetBlowerOutput = targetBlowerOutput;
    }



    public double getPos() {
		return masterArmMotor.getPosition().getValueAsDouble();
	}

    public double getVel() {
        return masterArmMotor.getVelocity().getValueAsDouble();
    }

    public double getEncoderPos() {
		return encoder.getAbsolutePosition();
	}

    public double getEncoderPosDeg() {
        double degrees = getEncoderPos() * 360 + ARM_ENCODER_TO_0_DEGREES;

        if (degrees < 0) {
            degrees += 360;
        }

        if (degrees > ARM_NEVER_POS) {
            degrees -= 360;
        }

        if (degrees < -90 && degrees > -270) {
            degrees += 360;
        }

        return degrees; 
    }

    private double getGravityOffset() {
        return POS_KG * Math.cos(Math.toRadians(getEncoderPosDeg()));
    }

    public static Arm getInstance() {
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }
}
