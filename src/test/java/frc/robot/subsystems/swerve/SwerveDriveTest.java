package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.io.IOException;
import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.MockitoAnnotations;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.NeoADIS16470;

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
    private SwerveDriveKinematics kDriveKinematics;

    private SwerveModule[] modules;

    private void initModules() {
        modules = new SwerveModule[] {
            mockFrontLeftModule,
            mockFrontRightModule,
            mockRearLeftModule,
            mockRearRightModule
        };
    }

    @BeforeEach
    public void setup() throws IOException, org.json.simple.parser.ParseException {
        MockitoAnnotations.openMocks(this);
        initModules();
        
        // Mock gyro methods
        when(mockGyro.getRawGyroObject()).thenReturn(mockRawGyroObject);
        when(mockRawGyroObject.getZAngle()).thenReturn(0.0);
        when(mockRawGyroObject.getYAngle()).thenReturn(0.0);
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(0));

        // Mock SwerveModule positions and states
        for (SwerveModule module : modules) {
            when(module.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
            when(module.getState()).thenReturn(new SwerveModuleState(1.0, new Rotation2d(0)));
        }

        // Create RobotConfig with adjusted module positions
        ModuleConfig moduleConfig = new ModuleConfig(
            10, // Example parameter
            10, // Example parameter
            10, // Example parameter
            new DCMotor(10, 10, 10, 10, 10, 1), // Example DCMotor configuration
            10, // Example parameter
            10  // Example parameter
        );
        RobotConfig robotConfig = new RobotConfig(
            10, // Mass in KG
            10, // Moment of Inertia in KG*M^2
            moduleConfig,
            new Translation2d(0.5, 0.5),
            new Translation2d(0.5, -0.5),
            new Translation2d(-0.5, 0.5),
            new Translation2d(-0.5, -0.5)
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

        // Initialize kinematics based on module positions from RobotConfig
        kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.5, 0.5),    // Front Left Module Position
            new Translation2d(0.5, -0.5),   // Front Right Module Position
            new Translation2d(-0.5, 0.5),   // Rear Left Module Position
            new Translation2d(-0.5, -0.5)   // Rear Right Module Position
        );
    }

    @Test
    public void testInitialization() {
        assertNotNull(swerveDrive);
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("driveTestProvider")
    public void testDrive(String scenario, double xSpeed, double ySpeed, double rot, double[] expectedAngles) {
        // Drive command
        swerveDrive.drive(xSpeed, ySpeed, rot, true, false);

        // Capture the SwerveModuleState objects set to each module
        ArgumentCaptor<SwerveModuleState> captor = ArgumentCaptor.forClass(SwerveModuleState.class);

        // Verify and capture desired states for all modules
        for (SwerveModule module : modules) {
            verify(module).setDesiredState(captor.capture());
        }

        // Assert that each module received the correct desired angle
        for (int i = 0; i < modules.length; i++) {
            assertEquals(
                expectedAngles[i],
                captor.getAllValues().get(i).angle.getRadians(),
                0.01,
                String.format("Module %d angle mismatch", i)
            );
        }

        // Reset interactions for the next test
        reset(modules);
    }

    private static Stream<org.junit.jupiter.params.provider.Arguments> driveTestProvider() {
        return Stream.of(
            org.junit.jupiter.params.provider.Arguments.of( // testDriveForward
                "testDriveForward", 1.0, 0.0, 0.0,
                new double[] {0.0, 0.0, 0.0, 0.0}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveLeft
                "testDriveLeft", 0.0, 1.0, 0.0,
                new double[] {Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveRight
                "testDriveRight", 0.0, -1.0, 0.0,
                new double[] {-Math.PI / 2, -Math.PI / 2, -Math.PI / 2, -Math.PI / 2}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveBackward
                "testDriveBackward", -1.0, 0.0, 0.0,
                new double[] {Math.PI, Math.PI, Math.PI, Math.PI}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveForwardRight
                "testDriveForwardRight", 1.0, -1.0, 0.0,
                new double[] {-Math.PI / 4, -Math.PI / 4, -Math.PI / 4, -Math.PI / 4}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveForwardLeft
                "testDriveForwardLeft", 1.0, 1.0, 0.0,
                new double[] {Math.PI / 4, Math.PI / 4, Math.PI / 4, Math.PI / 4}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveBackwardRight
                "testDriveBackwardRight", -1.0, -1.0, 0.0,
                new double[] {-3 * Math.PI / 4, -3 * Math.PI / 4, -3 * Math.PI / 4, -3 * Math.PI / 4}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveBackwardLeft
                "testDriveBackwardLeft", -1.0, 1.0, 0.0,
                new double[] {3 * Math.PI / 4, 3 * Math.PI / 4, 3 * Math.PI / 4, 3 * Math.PI / 4}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveRotationRight
                "testDriveRotationRight", 0.0, 0.0, -1.0,
                new double[] {-Math.PI / 4, -3 * Math.PI / 4, Math.PI / 4, 3 * Math.PI / 4}
            ),
            org.junit.jupiter.params.provider.Arguments.of( // testDriveRotationLeft
                "testDriveRotationLeft", 0.0, 0.0, 1.0,
                new double[] {3 * Math.PI / 4, Math.PI / 4, -3 * Math.PI / 4, -Math.PI / 4}
            )
        );
    }

    @Test
    public void testOdometryUpdate() {
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(0));
        for (SwerveModule module : modules) {
            when(module.getPosition()).thenReturn(new SwerveModulePosition(0, new Rotation2d(0)));
        }

        swerveDrive.periodic(); // Ensure odometry is updated

        Pose2d pose = swerveDrive.getPose();
        assertEquals(0.0, pose.getX(), 0.01);
        assertEquals(0.0, pose.getY(), 0.01);
        assertEquals(0.0, pose.getRotation().getDegrees(), 0.01);
    }

    @Test
    public void testPoseRetrievalAndReset() {
        Pose2d initialPose = new Pose2d(1.0, 1.0, new Rotation2d(Math.PI / 2));
        swerveDrive.resetOdometry(initialPose);

        Pose2d pose = swerveDrive.getPose();
        assertEquals(1.0, pose.getX(), 0.01);
        assertEquals(1.0, pose.getY(), 0.01);
        assertEquals(Math.PI / 2, pose.getRotation().getRadians(), 0.01);
    }

    @Test
    public void testGyroFunctionality() {
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(Math.PI / 4));
        assertEquals(Math.PI / 4, swerveDrive.getHeadingObject().getRadians(), 0.01);

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
        assertEquals(1.0, speeds.vxMetersPerSecond, 0.01, "Forward speed mismatch");
        assertEquals(0.0, speeds.vyMetersPerSecond, 0.01, "Sideways speed mismatch");
        assertEquals(0.0, speeds.omegaRadiansPerSecond, 0.01, "Angular speed mismatch");
    }

    @Test
    public void testFieldVisualization() {
        swerveDrive.periodic();
        assertNotNull(swerveDrive.fieldVisualization, "Field visualization should not be null");
    }

    // Add utilities for field orientation tests
    private static Stream<org.junit.jupiter.params.provider.Arguments> fieldOrientationTestCases() {
        return Stream.of(
            org.junit.jupiter.params.provider.Arguments.of(0.0, 0.0, 0.0),
            org.junit.jupiter.params.provider.Arguments.of(Math.PI / 2, 1.0, 0.0),
            org.junit.jupiter.params.provider.Arguments.of(Math.PI, 1.0, 0.0),
            org.junit.jupiter.params.provider.Arguments.of(3 * Math.PI / 2, 1.0, 0.0)
        );
    }

    // Add coordinate transformation verification helpers
    private void verifyTransformationMatrix(double expectedAngle, double actualAngle) {
        assertEquals(expectedAngle, actualAngle, 0.01, "Transformation matrix mismatch");
    }

    // Add gyro angle simulation capabilities
    private void simulateGyroAngle(double angleRadians) {
        double angleDegrees = Math.toDegrees(angleRadians);
        when(mockRawGyroObject.getZAngle()).thenReturn(angleDegrees);
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(angleRadians));
    }

    // Add tests for behavior during continuous rotation and when crossing quadrant boundaries
    @ParameterizedTest
    @MethodSource("fieldOrientationTestCases")
    public void testOrientationTransitions(double angle, double xSpeed, double ySpeed) {
        simulateGyroAngle(angle);
        
        // Compute target angle relative to the robot orientation
        double targetAngle = (xSpeed == 0.0 && ySpeed == 0.0) ? 
            0.0 : Math.atan2(ySpeed, xSpeed) - angle;
        
        // Normalize targetAngle to [-π, π]
        targetAngle = Math.atan2(Math.sin(targetAngle), Math.cos(targetAngle));
        
        swerveDrive.drive(xSpeed, ySpeed, 0.0, true, false);
    
        // Capture the SwerveModuleState objects set to each module
        ArgumentCaptor<SwerveModuleState> captor = ArgumentCaptor.forClass(SwerveModuleState.class);
    
        // Verify and capture desired states for all modules
        for (SwerveModule module : modules) {
            verify(module).setDesiredState(captor.capture());
        }
        
        // Verify that the module angles match the expected targetAngle
        for (SwerveModuleState state : captor.getAllValues()) {
            double actualAngle = state.angle.getRadians();
            // Normalize actualAngle to [-π, π]
            actualAngle = Math.atan2(Math.sin(actualAngle), Math.cos(actualAngle));
            assertEquals(targetAngle, actualAngle, 1e-6, "Transformation matrix mismatch");
        }
    }

    
}
