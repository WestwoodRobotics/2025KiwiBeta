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

import frc.robot.commands.swerve.driveCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;


public class SwerveDriveTest {

    @Mock
    private Gyro mockGyro;

    @Mock
    private ADIS16470_IMU mockRawGyroObject;

    @Mock
    private MAXSwerveModule mockFrontLeftModule;

    @Mock
    private MAXSwerveModule mockFrontRightModule;

    @Mock
    private MAXSwerveModule mockRearLeftModule;

    @Mock
    private MAXSwerveModule mockRearRightModule;

    @Mock
    private XboxController mockController;

    private SwerveDrive swerveDrive;

    private SwerveDriveKinematics kDriveKinematics;

    private MAXSwerveModule[] modules;

    private void initModules() {
        modules = new MAXSwerveModule[] {
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
        when(mockGyro.getRawRot2dYaw()).thenReturn(new Rotation2d(0)); // Add this line
        when(mockGyro.getRawRot2dRoll()).thenReturn(new Rotation2d(0)); // Add this line
        when(mockGyro.getRawRot2dPitch()).thenReturn(new Rotation2d(0)); // Add this line
        when(mockRawGyroObject.getAngle(IMUAxis.kZ)).thenReturn(0.0);
        when(mockRawGyroObject.getAngle(IMUAxis.kY)).thenReturn(0.0);
        when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(0));

        // Mock SwerveModule positions and states
        for (MAXSwerveModule module : modules) {
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
        for (MAXSwerveModule module : modules) {
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
        for (MAXSwerveModule module : modules) {
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

    @Test
    public void testFieldRelativeDrive() {
        double[] angles = {0.0, 90.0, 180.0, 270.0};
        for (double gyroAngle : angles) {
            // Mock both raw gyro angle and processed rotation
            when(mockRawGyroObject.getAngle(IMUAxis.kZ)).thenReturn(gyroAngle);
            when(mockGyro.getProcessedRot2dYaw()).thenReturn(new Rotation2d(Math.toRadians(gyroAngle)));

            // Command "forward" in field-relative terms
            swerveDrive.drive(1.0, 0.0, 0.0, true, false);

            // Capture states
            ArgumentCaptor<SwerveModuleState> captor = ArgumentCaptor.forClass(SwerveModuleState.class);
            for (MAXSwerveModule module : modules) {
                verify(module).setDesiredState(captor.capture());
            }

            // Expect each module to be angled correctly based on the gyro angle
            for (int i = 0; i < modules.length; i++) {
                double expectedAngle = -(gyroAngle * Math.PI/180.0) % (2*Math.PI);
                if (expectedAngle > Math.PI) expectedAngle -= 2*Math.PI;
                if (expectedAngle < -Math.PI) expectedAngle += 2*Math.PI;
                assertEquals(expectedAngle, captor.getAllValues().get(i).angle.getRadians(), 0.01,
                    "Field-relative mismatch at gyro angle: " + gyroAngle + ", module: " + i);
            }
            reset(modules);
        }
    }

    @Test
    public void testDriveCommand() {
        driveCommand driveCmd = new driveCommand(swerveDrive, mockController);

        // Simulate forward input
        when(mockController.getLeftX()).thenReturn(0.0);
        when(mockController.getLeftY()).thenReturn(-1.0);
        when(mockController.getRightX()).thenReturn(0.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {0.0, 0.0, 0.0, 0.0}, "Forward");

        // Simulate backward input
        when(mockController.getLeftY()).thenReturn(1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {Math.PI, Math.PI, Math.PI, Math.PI}, "Backward");

        // Simulate left input
        when(mockController.getLeftY()).thenReturn(0.0);
        when(mockController.getLeftX()).thenReturn(-1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2}, "Left");

        // Simulate right input
        when(mockController.getLeftX()).thenReturn(1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {-Math.PI / 2, -Math.PI / 2, -Math.PI / 2, -Math.PI / 2}, "Right");

        // Simulate forward-right diagonal input
        when(mockController.getLeftX()).thenReturn(1.0);
        when(mockController.getLeftY()).thenReturn(-1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {-Math.PI / 4, -Math.PI / 4, -Math.PI / 4, -Math.PI / 4}, "Forward-Right");

        // Simulate forward-left diagonal input
        when(mockController.getLeftX()).thenReturn(-1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {Math.PI / 4, Math.PI / 4, Math.PI / 4, Math.PI / 4}, "Forward-Left");

        // Simulate backward-right diagonal input
        when(mockController.getLeftX()).thenReturn(1.0);
        when(mockController.getLeftY()).thenReturn(1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {-3 * Math.PI / 4, -3 * Math.PI / 4, -3 * Math.PI / 4, -3 * Math.PI / 4}, "Backward-Right");

        // Simulate backward-left diagonal input
        when(mockController.getLeftX()).thenReturn(-1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {3 * Math.PI / 4, 3 * Math.PI / 4, 3 * Math.PI / 4, 3 * Math.PI / 4}, "Backward-Left");

        // Simulate rotation right input
        when(mockController.getLeftX()).thenReturn(0.0);
        when(mockController.getLeftY()).thenReturn(0.0);
        when(mockController.getRightX()).thenReturn(1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {-1 * Math.PI / 4, -3 * Math.PI / 4, Math.PI / 4, 3 * Math.PI / 4}, "Rotation-Right");

        // Simulate rotation left input
        when(mockController.getRightX()).thenReturn(-1.0);
        driveCmd.execute();
        verifyWheelAngles(new double[] {3 * Math.PI / 4, Math.PI / 4, -3 * Math.PI / 4, -Math.PI / 4}, "Rotation-Left");
        

    }

    private void verifyWheelAngles(double[] expectedAngles, String direction) {
        ArgumentCaptor<SwerveModuleState> captor = ArgumentCaptor.forClass(SwerveModuleState.class);
        for (MAXSwerveModule module : modules) {
            verify(module, atLeastOnce()).setDesiredState(captor.capture());
        }
        for (int i = 0; i < modules.length; i++) {
            assertEquals(expectedAngles[i], captor.getAllValues().get(i).angle.getRadians(), 0.01, direction + " - Module " + i + " angle mismatch");
        }
        reset(modules);
    }

    
}
