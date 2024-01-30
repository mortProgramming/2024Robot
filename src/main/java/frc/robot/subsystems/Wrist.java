package frc.robot.subsystems;

import static frc.robot.util.Constants.RobotSpecs.*;
import static frc.robot.util.Constants.Wrist.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Wrist extends SubsystemBase {
    
    private static Wrist wrist;

    private TalonFX wristMotor;

    private double wristSpeed;

    private double setpoint;

    private ProfiledPIDController wristPositionController;
    private ArmFeedforward wristPostionFeedForward;


    public Wrist() {
        wristMotor = new TalonFX(WRIST_MOTOR);
    
        wristPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        wristPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);
    }

    public void init() {
    //add motor initialization
    }

    /**
     * 
     */
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        wristMotor.set(wristSpeed);
    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    /**
     * 
     */
    public void setWristVelocity(double wristSpeed){
        this.wristSpeed = wristSpeed;
    }

    public void setSetPoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public double getVelocity(){
        return wristMotor.getVelocity().getValueAsDouble();
    }

    public double getPosition(){
        return wristMotor.getPosition().getValueAsDouble();
    }

    public boolean nearSetpoint(){
         return (Math.abs((wristPostionFeedForward.calculate(getPosition(), getVelocity()) + 
         wristPositionController.calculate(getPosition(),setpoint))) / MAX_VOLTAGE) < WRIST_NEAR_SETPOINT_ERROR;
    }

    public boolean isClear(){
        return true;
    }

    /**
     * 
     * @return
     */
    public static Wrist getInstance(){
        if (wrist==null){
            wrist = new Wrist();
        }
        return wrist;
    }
}
