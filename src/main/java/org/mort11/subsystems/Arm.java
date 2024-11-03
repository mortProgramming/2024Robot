package org.mort11.subsystems;

import static org.mort11.configuration.constants.PhysicalConstants.Arm.*;
import static org.mort11.configuration.constants.PIDConstants.Arm.*;
import static org.mort11.configuration.constants.PortConstants.Arm.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends SubsystemBase {
    
    private static Arm arm;

    //left is main motor
    private static TalonFX masterArmMotor;
    private static TalonFX followArmMotor;
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

        armPositionController = new ProfiledPIDController(POS_KP, POS_KI, POS_KD, 
            POS_CONSTRAINTS);

        blowerController = new PIDController(BLOWER_KP, BLOWER_KI, BLOWER_KD);
        

    }

    @Override
    public void periodic() {
        masterArmMotor.set(armSpeed);

        currentBlowerOutput += blowerController.calculate(currentBlowerOutput,targetBlowerOutput);
        blowerMotor.set(currentBlowerOutput);

        SmartDashboard.putNumber("Encoder Arm Pos Degrees", encoderToDegrees());
        SmartDashboard.putNumber("Arm Setpoint", setpoint);
        SmartDashboard.putNumber("arm output", armSpeed);
        SmartDashboard.putNumber("ActualArmMotorOutput", masterArmMotor.get());
        SmartDashboard.putNumber("Blower Value", currentBlowerOutput);
    }

    public void setArmVelocity(double armSpeed){
        this.armSpeed = armSpeed + getGravityOffset();
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;

        this.armSpeed = -armPositionController.calculate(encoderToDegrees(), setpoint) +
            getGravityOffset();
    }

    public void setBlowerTarget(double targetBlowerOutput){
        this.targetBlowerOutput = targetBlowerOutput;
    }



    public double getSetpoint(){
        return setpoint;
    }

    public double getVelocity() {
        return masterArmMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition() {
		return masterArmMotor.getPosition().getValueAsDouble();
	}

    // encoder
    public double getEncoderPosition() {
		return encoder.getAbsolutePosition();
	}

    //encoder
    public double encoderToDegrees() {
        double degrees = getEncoderPosition() * 360 + ARM_ENCODER_TO_0_DEGREES;
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

    public boolean nearSetpoint(){
        return encoderToDegrees() >= setpoint && encoderToDegrees() <= setpoint;
    }

    private double setPosition(double setpoint) {
		double output = (POS_KG * Math.cos(Math.toRadians(encoderToDegrees())))
        - armPositionController.calculate(encoderToDegrees(), setpoint);

        return output;
	}

    private double getGravityOffset() {
        return POS_KG * Math.cos(Math.toRadians(encoderToDegrees()));
    }

    public static Arm getInstance(){
        if (arm==null){
            arm = new Arm();

        }
        return arm;
    }
}
