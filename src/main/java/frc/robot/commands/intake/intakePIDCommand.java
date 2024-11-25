package frc.robot.commands.intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * The intakePIDCommand class is responsible for controlling the intake subsystem using a PID controller.
 * It sets the intake motor power based on the target RPM and the current RPM of the motor.
 */
public class intakePIDCommand extends Command{

    private double targetRPM;
    private Intake intake;
    private PIDController intakePIDController;
    
    /**
     * Constructs a new intakePIDCommand.
     * 
     * @param intake The intake subsystem used by this command.
     * @param targetRPM The target RPM for the intake motor.
     */
    public intakePIDCommand(Intake intake, double targetRPM){
        this.intake = intake;
        this.targetRPM = targetRPM;
        this.intakePIDController = intake.getPIDController();
        addRequirements(intake);
    }

    /**
     * Initializes the command by setting the PID controller's setpoint to the target RPM.
     */
    @Override
    public void initialize() {
        intakePIDController.setSetpoint(targetRPM);
    }

    /**
     * Executes the command by setting the intake motor power based on the PID controller's calculation.
     */
    @Override
    public void execute() {
        intake.setIntakePower(intakePIDController.calculate(intake.getRawMotorRPM()));
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return Always returns false, as this command never finishes on its own.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Ends the command by stopping the intake motor.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
