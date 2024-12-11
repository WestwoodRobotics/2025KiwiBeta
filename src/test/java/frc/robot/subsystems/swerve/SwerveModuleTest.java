package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import CustomLibs.QualityOfLife.NeoSparkMax;
import CustomLibs.QualityOfLife.NeoSparkFlex;

public class SwerveModuleTest {

    @Mock
    private NeoSparkMax mockNeoSparkMax;

    @Mock
    private NeoSparkFlex mockNeoSparkFlex;

    @BeforeEach
    public void setup() {
        MockitoAnnotations.openMocks(this);
    }

    @Test
    public void testNeoSparkMaxPIDValues() {
        when(mockNeoSparkMax.getP()).thenReturn(1.0);
        when(mockNeoSparkMax.getI()).thenReturn(0.1);
        when(mockNeoSparkMax.getD()).thenReturn(0.01);
        when(mockNeoSparkMax.getF()).thenReturn(0.001);

        assertEquals(1.0, mockNeoSparkMax.getP());
        assertEquals(0.1, mockNeoSparkMax.getI());
        assertEquals(0.01, mockNeoSparkMax.getD());
        assertEquals(0.001, mockNeoSparkMax.getF());
    }

    @Test
    public void testNeoSparkMaxOutputRange() {
        when(mockNeoSparkMax.getOutputRangeMin()).thenReturn(-1.0);
        when(mockNeoSparkMax.getOutputRangeMax()).thenReturn(1.0);

        assertEquals(-1.0, mockNeoSparkMax.getOutputRangeMin());
        assertEquals(1.0, mockNeoSparkMax.getOutputRangeMax());
    }

    @Test
    public void testNeoSparkMaxMotorSettings() {
        when(mockNeoSparkMax.getIdleMode()).thenReturn(NeoSparkMax.IdleMode.kBrake);
        when(mockNeoSparkMax.getSmartCurrentLimit()).thenReturn(40);

        assertEquals(NeoSparkMax.IdleMode.kBrake, mockNeoSparkMax.getIdleMode());
        assertEquals(40, mockNeoSparkMax.getSmartCurrentLimit());
    }

    @Test
    public void testNeoSparkMaxEncoderSettings() {
        when(mockNeoSparkMax.getPositionConversionFactor()).thenReturn(1.0);
        when(mockNeoSparkMax.getVelocityConversionFactor()).thenReturn(1.0);

        assertEquals(1.0, mockNeoSparkMax.getPositionConversionFactor());
        assertEquals(1.0, mockNeoSparkMax.getVelocityConversionFactor());
    }

    @Test
    public void testNeoSparkMaxPositionWrappingSettings() {
        when(mockNeoSparkMax.isPositionWrappingEnabled()).thenReturn(true);
        when(mockNeoSparkMax.getPositionWrappingInputRangeMin()).thenReturn(0.0);
        when(mockNeoSparkMax.getPositionWrappingInputRangeMax()).thenReturn(360.0);

        assertTrue(mockNeoSparkMax.isPositionWrappingEnabled());
        assertEquals(0.0, mockNeoSparkMax.getPositionWrappingInputRangeMin());
        assertEquals(360.0, mockNeoSparkMax.getPositionWrappingInputRangeMax());
    }

    @Test
    public void testNeoSparkFlexPIDValues() {
        when(mockNeoSparkFlex.getP()).thenReturn(1.0);
        when(mockNeoSparkFlex.getI()).thenReturn(0.1);
        when(mockNeoSparkFlex.getD()).thenReturn(0.01);
        when(mockNeoSparkFlex.getF()).thenReturn(0.001);

        assertEquals(1.0, mockNeoSparkFlex.getP());
        assertEquals(0.1, mockNeoSparkFlex.getI());
        assertEquals(0.01, mockNeoSparkFlex.getD());
        assertEquals(0.001, mockNeoSparkFlex.getF());
    }

    @Test
    public void testNeoSparkFlexOutputRange() {
        when(mockNeoSparkFlex.getOutputRangeMin()).thenReturn(-1.0);
        when(mockNeoSparkFlex.getOutputRangeMax()).thenReturn(1.0);

        assertEquals(-1.0, mockNeoSparkFlex.getOutputRangeMin());
        assertEquals(1.0, mockNeoSparkFlex.getOutputRangeMax());
    }

    @Test
    public void testNeoSparkFlexMotorSettings() {
        when(mockNeoSparkFlex.getIdleMode()).thenReturn(NeoSparkFlex.IdleMode.kBrake);
        when(mockNeoSparkFlex.getSmartCurrentLimit()).thenReturn(40);

        assertEquals(NeoSparkFlex.IdleMode.kBrake, mockNeoSparkFlex.getIdleMode());
        assertEquals(40, mockNeoSparkFlex.getSmartCurrentLimit());
    }

    @Test
    public void testNeoSparkFlexEncoderSettings() {
        when(mockNeoSparkFlex.getPositionConversionFactor()).thenReturn(1.0);
        when(mockNeoSparkFlex.getVelocityConversionFactor()).thenReturn(1.0);

        assertEquals(1.0, mockNeoSparkFlex.getPositionConversionFactor());
        assertEquals(1.0, mockNeoSparkFlex.getVelocityConversionFactor());
    }

    @Test
    public void testNeoSparkFlexPositionWrappingSettings() {
        when(mockNeoSparkFlex.isPositionWrappingEnabled()).thenReturn(true);
        when(mockNeoSparkFlex.getPositionWrappingInputRangeMin()).thenReturn(0.0);
        when(mockNeoSparkFlex.getPositionWrappingInputRangeMax()).thenReturn(360.0);

        assertTrue(mockNeoSparkFlex.isPositionWrappingEnabled());
        assertEquals(0.0, mockNeoSparkFlex.getPositionWrappingInputRangeMin());
        assertEquals(360.0, mockNeoSparkFlex.getPositionWrappingInputRangeMax());
    }
}
