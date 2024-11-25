package frc.robot.commands.preRoller;
import java.util.LinkedList;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.preRoller;

/**
 * The preRollerSenseCommand class is responsible for controlling the preRoller subsystem using a PID controller.
 * It monitors the current draw of the preRoller motor and stops the motor when a significant change in current is detected.
 */
public class preRollerSenseCommand extends Command {

    private LinkedList<Double> currentList = new LinkedList<>();
    private PIDController preRollerPIDController;
    private long timerDelayMS;
    private double currentDeltaThreshold;
    private double runningAverage;
    private preRoller preRoller;
    private int sampleSize;
    private double targetRPM;
    private boolean sampleSizeReached = false;
    private boolean timerStarted = false;

    private Timer timer = new Timer();

    /**
     * Constructs a new preRollerSenseCommand.
     * 
     * @param preRoller The preRoller subsystem used by this command.
     * @param targetRPM The target RPM for the preRoller motor.
     * @param timerDelay The delay in seconds before stopping the motor after detecting a significant current change.
     * @param currentDeltaThreshold The threshold for detecting a significant change in current.
     * @param sampleSize The number of current samples to use for calculating the running average.
     */
    public preRollerSenseCommand(preRoller preRoller, double targetRPM, double timerDelay, double currentDeltaThreshold, int sampleSize) {
        this.timerDelayMS = (long)(timerDelay * 1000);
        this.currentDeltaThreshold = currentDeltaThreshold;
        this.preRoller = preRoller; 
        this.preRollerPIDController = preRoller.getPIDController();
        this.sampleSize = sampleSize;
        this.targetRPM = targetRPM;
        addRequirements(preRoller);
    }

    /**
     * Initializes the command by resetting the current list, timer, and PID controller setpoint.
     */
    @Override
    public void initialize() {
        runningAverage = 0;
        currentList.clear();
        sampleSizeReached = false;
        timerStarted = false;
        timer.stop();
        timer.reset();
        preRollerPIDController.setSetpoint(targetRPM);
    }

    /**
     * Executes the command by adding the current draw to the list and updating the preRoller motor power.
     */
    @Override
    public void execute() {
        currentList.add(preRoller.getOutputCurrent());
        if (currentList.size() > sampleSize) {
            currentList.removeFirst();
            sampleSizeReached = true;
        }
        preRoller.setPreRollerPower(preRollerPIDController.calculate(preRoller.getRawMotorRPM()));
    }

    /**
     * Returns whether the command has finished.
     * 
     * @return True if the timer has elapsed after detecting a significant current change, false otherwise.
     */
    @Override
    public boolean isFinished() {
        if (sampleSizeReached) {
            calculateRunningAverage(currentList);
            double currentDelta = Math.abs(preRoller.getOutputCurrent() - runningAverage);
            if (currentDelta > currentDeltaThreshold) {
                if (!timerStarted) {
                    timerStarted = true;
                    timer.reset();
                    timer.start();
                }
            }
            return timer.hasElapsed(timerDelayMS / 1000.0);
        }
        return false;
    }

    /**
     * Ends the command by stopping the preRoller motor and resetting the current list and timer.
     * 
     * @param interrupted Whether the command was interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        preRoller.stopPreRoller();
        preRoller.setHoldingNote(true);
        runningAverage = 0;
        currentList.clear();
        timerStarted = false;
        timer.stop();
        timer.reset();
        System.out.println("Finished Command");
    }

    /**
     * Calculates the running average of the current draw.
     * 
     * @param currentList The list of current samples.
     */
    private void calculateRunningAverage(LinkedList<Double> currentList) {
        double sum = 0;
        for (double current : currentList) {
            sum += current;
        }
        runningAverage = sum / currentList.size();
    }
}