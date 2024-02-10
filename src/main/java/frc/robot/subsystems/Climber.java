package frc.robot.subsystems;

import static frc.robot.util.Constants.Climber.*;
import static frc.robot.util.Constants.RobotSpecs.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class Climber extends SubsystemBase {
    
    private static Climber climber;

    //left is main motor
    private CANSparkMax masterClimberMotor;
    private CANSparkMax followClimberMotor;

    private double climberSpeed;
    private double setpoint;

    private ProfiledPIDController climberPositionController;
    private ArmFeedforward climberPostionFeedForward;

    public Climber() {
        masterClimberMotor = new CANSparkMax(MASTER_CLIMBER_MOTOR, MotorType.kBrushless);
        followClimberMotor = new CANSparkMax(FOLLOW_CLIMBER_MOTOR, MotorType.kBrushless);

        followClimberMotor.follow(masterClimberMotor, true);

        climberPositionController = new ProfiledPIDController(POSITION_PID_P, POSITION_PID_I, POSITION_PID_D, 
        new Constraints(POSITION_PID_V, POSITION_PID_A));

        climberPostionFeedForward = new ArmFeedforward(POSITION_FF_S, POSITION_FF_G, POSITION_FF_V, POSITION_FF_A);
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
        masterClimberMotor.set(climberSpeed);
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
    public void setClimberVelocity(double climberSpeed){
        this.climberSpeed = climberSpeed;
    }

    public void setSetPoint(double setpoint){
        this.setpoint = setpoint;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public double getVelocity() {
        return masterClimberMotor.getEncoder().getVelocity();
    }

    public double getPosition() {
        return masterClimberMotor.getEncoder().getPosition();
	}

    public boolean nearSetpoint(){
        return (Math.abs(climberPostionFeedForward.calculate(getPosition(), getVelocity()) + 
         climberPositionController.calculate(getPosition(),setpoint)) / MAX_VOLTAGE) < CLIMBER_NEAR_SETPOINT_ERROR;
    }

   public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    /**
     * 
     * @return
     */
    public static Climber getInstance(){
        if (climber==null){
            climber = new Climber();
        }
        return climber;
    }
}
