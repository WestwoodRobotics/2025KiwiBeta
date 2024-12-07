package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.sensors.NeoADIS16470;

import java.io.IOException;

public class SwerveDriveTest {

    @Mock
    private Gyro mockGyro;

    @Mock
    private NeoADIS16470 mockRawGyroObject;

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

        // Mock gyro methods
        when(mockGyro.getRawGyroObject()).thenReturn(mockRawGyroObject);
        when(mockRawGyroObject.getZAngle()).thenReturn(0.0);
        when(mockRawGyroObject.getXAngle()).thenReturn(0.0);
        when(mockRawGyroObject.getYAngle()).thenReturn(0.0);
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(0));

        // Mock SwerveModule positions
        when(mockFrontLeftModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockFrontRightModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockRearLeftModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        when(mockRearRightModule.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));

        // Mock SwerveModule states
        when(mockFrontLeftModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockFrontRightModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockRearLeftModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        when(mockRearRightModule.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));

        // Create RobotConfig
        ModuleConfig moduleConfig = new ModuleConfig(10, 10, 10, new DCMotor(10, 10, 10, 10, 10, 1), 10, 10);
        RobotConfig robotConfig = new RobotConfig(
            10, // Mass in KG
            10, // Moment of Inertia in KG*M^2
            moduleConfig,
            new Translation2d(10, 10),
            new Translation2d(10, -10),
            new Translation2d(-10, 10),
            new Translation2d(-10, -10)
        );

        // Initialize SwerveDrive with isTestMode=true to bypass test mode conditionals
        swerveDrive = new SwerveDrive(
            mockGyro,
            mockFrontLeftModule,
            mockFrontRightModule,
            mockRearLeftModule,
            mockRearRightModule,
            robotConfig,
            true
        );
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

        swerveDrive.periodic(); // Ensure odometry is updated

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
        // Set up module states with known speeds and angles
        SwerveModuleState frontLeftState = new SwerveModuleState(1.0, new Rotation2d(0));
        SwerveModuleState frontRightState = new SwerveModuleState(1.0, new Rotation2d(0));
        SwerveModuleState rearLeftState = new SwerveModuleState(1.0, new Rotation2d(0));
        SwerveModuleState rearRightState = new SwerveModuleState(1.0, new Rotation2d(0));
        
        // Mock each module's getState() method to return our test states
        when(mockFrontLeftModule.getState()).thenReturn(frontLeftState);
        when(mockFrontRightModule.getState()).thenReturn(frontRightState);
        when(mockRearLeftModule.getState()).thenReturn(rearLeftState);
        when(mockRearRightModule.getState()).thenReturn(rearRightState);
        
        // Get chassis speeds and verify them
        ChassisSpeeds speeds = swerveDrive.getRobotRelativeSpeeds();
        assertNotNull(speeds);
        
        // Due to swerve drive kinematics, when all modules are driving forward at 1.0 m/s,
        // the robot's overall forward speed should be 1.0 m/s
        assertEquals(1.0, speeds.vxMetersPerSecond, 0.01);
        assertEquals(0.0, speeds.vyMetersPerSecond, 0.01); // Sideways speed should be 0
        assertEquals(0.0, speeds.omegaRadiansPerSecond, 0.01); // Angular speed should be 0
    }

    @Test
    public void testFieldVisualization() {
        swerveDrive.periodic();
        assertNotNull(swerveDrive.fieldVisualization);
    }
}