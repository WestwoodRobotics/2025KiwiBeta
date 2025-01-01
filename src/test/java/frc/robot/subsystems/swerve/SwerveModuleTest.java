package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;


import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;


import CustomLibs.QualityOfLife.NeoSparkMax;
import CustomLibs.QualityOfLife.NeoSparkFlex;
import CustomLibs.QualityOfLife.NeoSparkLowLevel.MotorType;
import CustomLibs.QualityOfLife.NeoSparkBaseConfig.IdleMode;
import CustomLibs.QualityOfLife.NeoMAXMotionConfig.MAXMotionPositionMode;

public class SwerveModuleTest {
    private static final double DELTA = 1e-6;
    private static int incrementCANID = 1;
    @Mock
    private NeoSparkMax realNeoSparkMax;

    @Mock
    private NeoSparkFlex realNeoSparkFlex;

    @BeforeEach
    public void setup() {

        realNeoSparkMax = new NeoSparkMax(incrementCANID, MotorType.kBrushless);
        realNeoSparkFlex = new NeoSparkFlex(incrementCANID+1, MotorType.kBrushless);
        incrementCANID += 2;
    }

    @Test
    public void testNeoSparkMaxPIDValues() {
        realNeoSparkMax.setPIDF(1.0, 0.1, 0.01, 0.001);
        realNeoSparkMax.configure(realNeoSparkMax.getCurrentConfig());
        assertEquals(1.0, realNeoSparkMax.getP(), DELTA);
        assertEquals(0.1, realNeoSparkMax.getI(), DELTA);
        assertEquals(0.01, realNeoSparkMax.getD(), DELTA);
        assertEquals(0.001, realNeoSparkMax.getF(), DELTA);
    }

    @Test
    public void testNeoSparkMaxOutputRange() {
        realNeoSparkMax.setOutputRange(-1.0, 1.0);
        realNeoSparkMax.configure(realNeoSparkMax.getCurrentConfig());

        assertEquals(-1.0, realNeoSparkMax.getOutputRangeMin());
        assertEquals(1.0, realNeoSparkMax.getOutputRangeMax());
    }

    @Test
    public void testNeoSparkMaxMotorSettings() {
        realNeoSparkMax.setIdleMode(IdleMode.kBrake);
        realNeoSparkMax.setSmartCurrentLimit(40);
        assertEquals(IdleMode.kBrake, realNeoSparkMax.getIdleMode());
        assertEquals(40, realNeoSparkMax.getSmartCurrentLimit());
    }

    @Test
    public void testNeoSparkMaxEncoderSettings() {

        realNeoSparkMax.setPositionConversionFactor(1.0);
        realNeoSparkMax.setVelocityConversionFactor(1.0);


        assertEquals(1.0, realNeoSparkMax.getPositionConversionFactor());
        assertEquals(1.0, realNeoSparkMax.getVelocityConversionFactor());
    }

    @Test
    public void testNeoSparkMaxPositionWrappingSettings() {
        realNeoSparkMax.setPositionWrappingEnabled(true);
        realNeoSparkMax.setPositionWrappingInputRange(0.0, 360.0);
        assertTrue(realNeoSparkMax.isPositionWrappingEnabled());
        assertEquals(0.0, realNeoSparkMax.getPositionWrappingInputRangeMin());
        assertEquals(360.0, realNeoSparkMax.getPositionWrappingInputRangeMax());
    }

    @Test
    public void testNeoSparkFlexPIDValues() {
        realNeoSparkFlex.setPIDF(1.0, 0.1, 0.01, 0.001);
        realNeoSparkFlex.configure(realNeoSparkFlex.getCurrentConfig());

        assertEquals(1.0, realNeoSparkFlex.getP(), DELTA);
        assertEquals(0.1, realNeoSparkFlex.getI(), DELTA);
        assertEquals(0.01, realNeoSparkFlex.getD(), DELTA);
        assertEquals(0.001, realNeoSparkFlex.getF(), DELTA);
    }

    @Test
    public void testNeoSparkFlexOutputRange() {
        realNeoSparkFlex.setOutputRange(-1.0, 1.0);

        assertEquals(-1.0, realNeoSparkFlex.getOutputRangeMin());
        assertEquals(1.0, realNeoSparkFlex.getOutputRangeMax());
    }

    @Test
    public void testNeoSparkFlexMotorSettings() {
        realNeoSparkFlex.setIdleMode(IdleMode.kBrake);
        realNeoSparkFlex.setSmartCurrentLimit(40);
        assertEquals(IdleMode.kBrake, realNeoSparkFlex.getIdleMode());
        assertEquals(40, realNeoSparkFlex.getSmartCurrentLimit());
    }

    @Test
    public void testNeoSparkFlexEncoderSettings() {
        realNeoSparkFlex.setPositionConversionFactor(1.0);
        realNeoSparkFlex.setVelocityConversionFactor(1.0);
        assertEquals(1.0, realNeoSparkFlex.getPositionConversionFactor());
        assertEquals(1.0, realNeoSparkFlex.getVelocityConversionFactor());
    }

    @Test
    public void testNeoSparkFlexPositionWrappingSettings() {
        realNeoSparkFlex.setPositionWrappingEnabled(true);
        realNeoSparkFlex.setPositionWrappingInputRange(0.0, 360.0);
        assertTrue(realNeoSparkFlex.isPositionWrappingEnabled());
        assertEquals(0.0, realNeoSparkFlex.getPositionWrappingInputRangeMin());
        assertEquals(360.0, realNeoSparkFlex.getPositionWrappingInputRangeMax());
    }


    @Test
    public void testNeoSparkMaxReconfiguration() {
        realNeoSparkMax.setPIDF(1.0, 0.1, 0.01, 0.001);
        realNeoSparkMax.configure(realNeoSparkMax.getCurrentConfig());
        
        // Reconfigure with new PIDF values
        realNeoSparkMax.setPIDF(2.0, 0.2, 0.02, 0.002);
        realNeoSparkMax.configure(realNeoSparkMax.getCurrentConfig());
        
        assertEquals(2.0, realNeoSparkMax.getP(), DELTA);
        assertEquals(0.2, realNeoSparkMax.getI(), DELTA);
        assertEquals(0.02, realNeoSparkMax.getD(), DELTA);
        assertEquals(0.002, realNeoSparkMax.getF(), DELTA);
    }
    
    @Test
    public void testNeoSparkFlexReconfiguration() {
        realNeoSparkFlex.setPIDF(1.0, 0.1, 0.01, 0.001);
        realNeoSparkFlex.configure(realNeoSparkFlex.getCurrentConfig());
        
        // Reconfigure with new PIDF values
        realNeoSparkFlex.setPIDF(2.0, 0.2, 0.02, 0.002);
        realNeoSparkFlex.configure(realNeoSparkFlex.getCurrentConfig());
        
        assertEquals(2.0, realNeoSparkFlex.getP(), DELTA);
        assertEquals(0.2, realNeoSparkFlex.getI(), DELTA);
        assertEquals(0.02, realNeoSparkFlex.getD(), DELTA);
        assertEquals(0.002, realNeoSparkFlex.getF(), DELTA);
    }

    @Test
    public void testNeoSparkMaxIdleModeTransitions() {
        realNeoSparkMax.setIdleMode(IdleMode.kBrake);
        assertEquals(IdleMode.kBrake, realNeoSparkMax.getIdleMode());
        
        realNeoSparkMax.setIdleMode(IdleMode.kCoast);
        assertEquals(IdleMode.kCoast, realNeoSparkMax.getIdleMode());
    }
    
    @Test
    public void testNeoSparkFlexIdleModeTransitions() {
        realNeoSparkFlex.setIdleMode(IdleMode.kBrake);
        assertEquals(IdleMode.kBrake, realNeoSparkFlex.getIdleMode());
        
        realNeoSparkFlex.setIdleMode(IdleMode.kCoast);
        assertEquals(IdleMode.kCoast, realNeoSparkFlex.getIdleMode());
    }

    @Test
    public void testNeoSparkMaxPositionWrappingBehavior() {
        // Enable position wrapping with valid range
        realNeoSparkMax.setPositionWrappingEnabled(true);
        realNeoSparkMax.setPositionWrappingInputRange(0.0, 360.0);
        assertTrue(realNeoSparkMax.isPositionWrappingEnabled());
        assertEquals(0.0, realNeoSparkMax.getPositionWrappingInputRangeMin());
        assertEquals(360.0, realNeoSparkMax.getPositionWrappingInputRangeMax());
        
        // Disable position wrapping
        realNeoSparkMax.setPositionWrappingEnabled(false);
        assertFalse(realNeoSparkMax.isPositionWrappingEnabled());
    }
    
    @Test
    public void testNeoSparkFlexPositionWrappingBehavior() {
        // Enable position wrapping with valid range
        realNeoSparkFlex.setPositionWrappingEnabled(true);
        realNeoSparkFlex.setPositionWrappingInputRange(0.0, 360.0);
        assertTrue(realNeoSparkFlex.isPositionWrappingEnabled());
        assertEquals(0.0, realNeoSparkFlex.getPositionWrappingInputRangeMin());
        assertEquals(360.0, realNeoSparkFlex.getPositionWrappingInputRangeMax());
        
        // Disable position wrapping
        realNeoSparkFlex.setPositionWrappingEnabled(false);
        assertFalse(realNeoSparkFlex.isPositionWrappingEnabled());
    }

    @Test
    public void testNeoSparkMaxConcurrentConfiguration() throws InterruptedException {
        Runnable configureTask = () -> {
            for (int i = 0; i < 1000; i++) {
                realNeoSparkMax.setPIDF(i, i * 0.1, i * 0.01, i * 0.001);
            }
        };
        
        Thread thread1 = new Thread(configureTask);
        Thread thread2 = new Thread(configureTask);
        
        thread1.start();
        thread2.start();
        
        thread1.join();
        thread2.join();
        
        // Verify final state is consistent
        assertTrue(realNeoSparkMax.getP() >= 0);
        assertTrue(realNeoSparkMax.getI() >= 0);
        assertTrue(realNeoSparkMax.getD() >= 0);
        assertTrue(realNeoSparkMax.getF() >= 0);
    }

    @Test
    public void testNeoSparkFlexConcurrentConfiguration() throws InterruptedException {
        Runnable configureTask = () -> {
            for (int i = 0; i < 1000; i++) {
                realNeoSparkFlex.setPIDF(i, i * 0.1, i * 0.01, i * 0.001);
            }
        };
        
        Thread thread1 = new Thread(configureTask);
        Thread thread2 = new Thread(configureTask);
        
        thread1.start();
        thread2.start();
        
        thread1.join();
        thread2.join();
        
        // Verify final state is consistent
        assertTrue(realNeoSparkFlex.getP() >= 0);
        assertTrue(realNeoSparkFlex.getI() >= 0);
        assertTrue(realNeoSparkFlex.getD() >= 0);
        assertTrue(realNeoSparkFlex.getF() >= 0);
    }

    @Test
    public void testNeoSparkMaxMAXMotionProfile() {
        realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().setMAXMotionMaxAcceleration(3.14);
        realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().setMAXMotionMaxVelocity(3.14);
        realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().setAllowedClosedLoopError(3.14);
        realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().setMAXMotionPositionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        realNeoSparkMax.configure(realNeoSparkMax.getCurrentConfig());

        // assertEquals(1000, realNeoSparkMax.getMotionProfileCruiseVelocity());
        // assertEquals(1000, realNeoSparkMax.getMotionProfileAcceleration());

        assertEquals(3.14, realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().getMAXMotionMaxAcceleration(), DELTA);
        assertEquals(3.14, realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().getMAXMotionMaxVelocity(), DELTA);
        assertEquals(3.14, realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().getAllowedClosedLoopError(), DELTA);
        assertEquals(MAXMotionPositionMode.kMAXMotionTrapezoidal, realNeoSparkMax.getCurrentConfig().getClosedLoopConfig().getMAXMotionPositionMode());
    }   

    
}
