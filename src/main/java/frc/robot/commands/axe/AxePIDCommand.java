package frc.robot.commands.axe;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.axe.Axe;

public class AxePIDCommand extends Command {
    
    private double targetPosition;
    private Axe axe;
    private PIDController axePIDController;

    /*
     * @param axe The axe subsystem used by this command.
     * @param targetPosition The target position for the axe.
     */
    public AxePIDCommand(Axe axe, double targetPosition){
        this.axe = axe;
        this.targetPosition = targetPosition;
        this.axePIDController = axe.getPIDController();
        addRequirements(axe);
    }

    @Override
    public void initialize() {
        axePIDController.setSetpoint(targetPosition);
    }

    @Override
    public void execute() {
        axe.setAxePower(axePIDController.calculate(axe.getAxePosition()));
    }

    @Override
    public boolean isFinished() {
        return false; // the axe lives forever
    }

    @Override
    public void end(boolean interrupted) {
        axe.setAxePower(0);
    }
}
