package org.mort11.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Elevator extends Subsystembase {
    private CANSParkMax elevatorMotor;
    private static Elevator findElevator;
        private CANSparkMax elevatorMotor;
    public void setMotor(double value)
        elevatorMotor.set(value);
    public static Elevator getInstance(){
        if (Elevator==null)
            Elevator=new Elevator;
        return Elevator
    }
        


}
