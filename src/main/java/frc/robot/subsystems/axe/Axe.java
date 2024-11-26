package frc.robot.subsystems.axe;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxeConstants;

public class Axe extends SubsystemBase {

    private SparkMax axeMotor;
    private PIDController axePIDController;
    private double encoderOffset;

    public Axe() {
        this.axeMotor = new SparkMax(AxeConstants.kAxeMotorPort, SparkMax.MotorType.kBrushless);
        this.axePIDController = new PIDController(AxeConstants.kP, AxeConstants.kI, AxeConstants.kD);
        
        // Create and apply motor configuration
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        
        // Apply the configuration to the motor
        axeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
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
        //System.out.println(this.getAxePosition());
    }
}