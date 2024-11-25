package frc.robot.subsystems.axe;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxeConstants;

public class Axe extends SubsystemBase {

    private CANSparkMax axeMotor;
    private PIDController axePIDController;
    private double encoderOffset;

    public Axe() {
        this.axeMotor = new CANSparkMax(AxeConstants.kAxeMotorPort, MotorType.kBrushless);
        this.axePIDController = new PIDController(AxeConstants.kP, AxeConstants.kI, AxeConstants.kD);
        axeMotor.setIdleMode(IdleMode.kCoast);
        encoderOffset = 0;
    }

    public void setAxePower(double power) {
        axeMotor.set(power);
    }

    public double getAxePosition() {
        return axeMotor.getEncoder().getPosition() - encoderOffset;
    }

    public double getRawAxePosition(){
        return axeMotor.getEncoder().getPosition();
    }

    public PIDController getPIDController() {
        return axePIDController;
    }

    public void resetEncoder(){
        encoderOffset = axeMotor.getEncoder().getPosition();
    }

    @Override
    public void periodic(){
        System.out.println(this.getAxePosition());
    }
    
}
