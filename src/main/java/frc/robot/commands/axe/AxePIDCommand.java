package frc.robot.commands.axe;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;

import CustomLibs.QualityOfLife.NeoSparkBase;
import CustomLibs.QualityOfLife.NeoSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AxeConstants;
import frc.robot.subsystems.axe.Axe;

public class AxePIDCommand extends Command {
    
    private double targetPosition;
    private Axe axe;
    private NeoSparkMax axeMotor;

    /*
     * @param axe The axe subsystem used by this command.
     * @param targetPosition The target position for the axe.
     */
    public AxePIDCommand(Axe axe, double targetPosition){
        this.axe = axe;
        this.targetPosition = targetPosition;
        this.axeMotor = axe.getAxeMotor();
        addRequirements(axe);
    }

    @Override
    public void initialize() {

        axeMotor.getCurrentConfig().getClosedLoopConfig().setMAXMotionMaxAcceleration(AxeConstants.kMAXMotionMaxAcceleration); // Set to an appropriate velocity value
        axeMotor.getCurrentConfig().getClosedLoopConfig().setMAXMotionMaxVelocity(AxeConstants.kMAXMotionMaxVelocity);
        axeMotor.configCurrentConfig();
        axeMotor.getClosedLoopController().setReference(targetPosition, NeoSparkBase.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false; // the axe lives forever
        
    }

    @Override
    public void end(boolean interrupted) {
        axeMotor.getClosedLoopController().setReference(0, NeoSparkBase.ControlType.kDutyCycle);
        axe.setAxePower(0);
    }
}
