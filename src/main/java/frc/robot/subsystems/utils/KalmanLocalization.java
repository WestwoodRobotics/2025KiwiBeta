package frc.robot.subsystems.utils;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.Timer;

public class KalmanLocalization {

    public Matrix<N7, N1> state;
    public Matrix<N7, N7> cov;

    public KalmanFilter<N7, N1, N6> filter;

    public double curr_time;

    public Pose2d startingPose;

    public Pose2d currentPose;

    public Twist2d deltaTwist;

    public double dx;
    public double dy;
    public double dtheta;


    public KalmanLocalization(Pose2d pose2d){
        this(pose2d.getX(),pose2d.getY(), pose2d.getRotation().getRadians());
    }

    public KalmanLocalization(
        double start_x,
        double start_y,
        double start_theta
    ) {
        state = new Matrix<N7, N1>(new SimpleMatrix(7, 1));
        state.set(0, 0, start_x);
        state.set(1, 0, start_y);
        state.set(2, 0, start_theta);
        state.set(3, 0, 0);
        state.set(4, 0, 0);
        state.set(5, 0, 0);
        state.set(6, 0, 0);

        cov = new Matrix<N7, N7>(new SimpleMatrix(7, 7));
            new double[][]{
                {0.1, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0.1, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0.1, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0.01, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0.01, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0.01, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0.001, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0.001, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0.1},
            }
        );
        filter = new KalmanFilter<>(state, cov);

        curr_time = Timer.getFPGATimestamp();
    }

    private Matrix<N7, N7> getUpdateMatrix(double dt) {
        return new Matrix<N7, N7>(new SimpleMatrix(7, 7)));
    }
    private Matrix<N9, N8> getControlMatrix(double dt) {
        return new Matrix<N9, N8>(new SimpleMatrix(
            new double[][]{
                {0},{0},{0},{0},{0},{0},{0},{0},{0},
            }
        ));
    }

    private Matrix<N6, N9> getSensorMatrix(double dt) {
        double theta = getTheta();
        return new Matrix<N6, N9>(new SimpleMatrix(
            new double[][]{
                {0, 0, 0, Math.cos(-theta)/dt, Math.sin(-theta)/dt, 0, 0, 0, 0},
                {0, 0, 0, -Math.sin(-theta)/dt, Math.cos(-theta)/dt, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 1/dt, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, Math.cos(-theta), Math.sin(-theta), 0},
                {0, 0, 0, 0, 0, 0, -Math.sin(-theta), Math.cos(-theta), 0},
                {0, 0, 0, 0, 0, 1, 0, 0, 1}
            }
        ));
    }

    private Matrix<N6, N6> getSensorCovariance(double dt, double speed) {
        return new Matrix<N6, N6>(new SimpleMatrix(
            new double[][]{
                {1e-6 + 0.01*speed, 0, 0, 0, 0, 0},
                {0, 1e-6 + 0.01*speed, 0, 0, 0, 0},
                {0, 0, 1e-6 + 0.01*speed, 0, 0, 0},
                {0, 0, 0, 0.001, 0, 0},
                {0, 0, 0, 0, 0.001, 0},
                {0, 0, 0, 0, 0, 0.001}
            }
        ));
    }


    private Matrix<N9, N9> getProcessCovariance(double dt) {
        return new Matrix<N9, N9>(new SimpleMatrix(
            new double[][]{
                {0.01, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0.01, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0.01, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0.01, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0.01, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0.01, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0.01, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0.01, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0.001},
            }
        ));
    }

    public void update(
        double odometry_dx,
        double odometry_dy,
        double odometry_dtheta,
        double imu_x_accel,
        double imu_y_accel,
        double gyro_dtheta,
        double speed
    ) {
        currentPose = new Pose2d(new Translation2d(odometry_dx, odometry_dy), new Rotation2d(odometry_dtheta));
        deltaTwist = startingPose.log(currentPose);
        dx = deltaTwist.dx;
        dy = deltaTwist.dy;
        dtheta = deltaTwist.dtheta;
        double next_time = Timer.getFPGATimestamp();
        double dt = next_time - curr_time;
        curr_time = next_time;

        filter.aPrioriUpdate(
            new Matrix<N1, N1>(new SimpleMatrix(new double[]{0})),
            getUpdateMatrix(dt),
            getProcessCovariance(dt),
            getControlMatrix(dt)
        );
        Matrix <N6, N1> sensor_input = new Matrix<N6, N1>(new SimpleMatrix(
            new double[]{odometry_dx, odometry_dy, odometry_dtheta, imu_x_accel, imu_y_accel, gyro_dtheta}
        ));
        filter.aPosteriorUpdate(
            sensor_input,
            getSensorCovariance(dt ,speed),
            getSensorMatrix(dt),
            N9.instance
        );
        state = filter.getState();
        cov = filter.getCovariance();
    }

    public double getX(){
        return state.get(0, 0);
    }
    public double getY(){
        return state.get(1, 0);
    }
    public double getTheta(){
        return state.get(2, 0);
    }
    public double getXVel(){
        return state.get(3, 0);
    }
    public double getYVel(){
        return state.get(4, 0);
    }
    public double getThetaVel(){
        return state.get(5, 0);
    }
    public double getXAccel(){
        return state.get(6, 0);
    }
    public double getYAccel(){
        return state.get(7, 0);
    }
    public double getGyroBias(){
        return state.get(8, 0);
    }

    public Pose2d getPoseMeters(){
        return new Pose2d(new Translation2d(this.getX(), this.getY()), new Rotation2d(this.getTheta()));
    }

    
        
}


