package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.io.IOException;

public class SwerveDriveTest {

    @Mock
    private Gyro mockGyro;

    @Mock
    private SwerveModule mockFrontLeftModule;

    @Mock
    private SwerveModule mockFrontRightModule;

    @Mock
    private SwerveModule mockRearLeftModule;

    @Mock
    private SwerveModule mockRearRightModule;

    private SwerveDrive swerveDrive;

    @BeforeEach
    public void setup() throws IOException, org.json.simple.parser.ParseException {
        MockitoAnnotations.openMocks(this);
        swerveDrive = new SwerveDrive(mockGyro, mockFrontLeftModule, mockFrontRightModule, mockRearLeftModule, mockRearRightModule);
    }

    @Test
    public void testInitialization() {
        assertNotNull(swerveDrive);
    }

    @Test
    public void testDrive() {
        swerveDrive.drive(1.0, 0.0, 0.0, true, false);
        verify(mockFrontLeftModule).setDesiredState(any(SwerveModuleState.class));
        verify(mockFrontRightModule).setDesiredState(any(SwerveModuleState.class));
        verify(mockRearLeftModule).setDesiredState(any(SwerveModuleState.class));
        verify(mockRearRightModule).setDesiredState(any(SwerveModuleState.class));
    }

    @Test
    public void testOdometryUpdate() {
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(0));
        when(mockFrontLeftModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockFrontRightModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockRearLeftModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockRearRightModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));

        swerveDrive.periodic();

        Pose2d pose = swerveDrive.getPose();
        assertEquals(0, pose.getX());
        assertEquals(0, pose.getY());
        assertEquals(0, pose.getRotation().getDegrees());
    }

    @Test
    public void testPoseRetrievalAndReset() {
        Pose2d initialPose = new Pose2d(1.0, 1.0, new Rotation2d(Math.PI / 2));
        swerveDrive.resetOdometry(initialPose);

        Pose2d pose = swerveDrive.getPose();
        assertEquals(1.0, pose.getX());
        assertEquals(1.0, pose.getY());
        assertEquals(Math.PI / 2, pose.getRotation().getRadians());
    }

    @Test
    public void testGyroFunctionality() {
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(Math.PI / 4));
        assertEquals(Math.PI / 4, swerveDrive.getHeadingObject().getRadians());

        swerveDrive.resetGyro();
        verify(mockGyro).reset();
    }

    @Test
    public void testModuleStateManagement() {
        SwerveModuleState[] states = {
            new SwerveModuleState(1.0, new Rotation2d(0)),
            new SwerveModuleState(1.0, new Rotation2d(Math.PI / 2)),
            new SwerveModuleState(1.0, new Rotation2d(Math.PI)),
            new SwerveModuleState(1.0, new Rotation2d(3 * Math.PI / 2))
        };

        swerveDrive.setModuleStates(states);
        verify(mockFrontLeftModule).setDesiredState(states[0]);
        verify(mockFrontRightModule).setDesiredState(states[1]);
        verify(mockRearLeftModule).setDesiredState(states[2]);
        verify(mockRearRightModule).setDesiredState(states[3]);
    }

    @Test
    public void testChassisSpeeds() {
        when(mockFrontLeftModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockFrontRightModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockRearLeftModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockRearRightModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));

        assertNotNull(swerveDrive.getRobotRelativeSpeeds());
    }

    @Test
    public void testFieldVisualization() {
        swerveDrive.periodic();
        assertNotNull(swerveDrive.fieldVisualization);
    }
}
