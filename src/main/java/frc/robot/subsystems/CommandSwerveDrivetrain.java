package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;
/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight-cg");
    Pigeon2  m_pigeon = new Pigeon2(0, "1599-B");

    //NetworkTable m_limelightRear = NetworkTableInstance.getDefault().getTable("limelight-back");
    //NetworkTable m_limelightFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    NetworkTable table = ntInstance.getTable("Pose");
    NetworkTable poseTable = ntInstance.getTable("MyPose");
    @SuppressWarnings("unused")
    private final SwerveRequest.ApplyRobotSpeeds m_ApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final PIDController m_pathXController = new PIDController(10, 0, 0);
    private final PIDController m_pathYController = new PIDController(10, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();



    public Rotation2d getHeading(){

        return new Rotation2d(m_pigeon.getYaw().getValue());
    }

    SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
        Constants.swervePoseEstimatorKinematic,
        getHeading(),
        getModulePositions(),
        new Pose2d()

    );


    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

/**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    
    /* private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    } */

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> this.getState().Pose,
            this::resetPose,
            this::followPath,
            false,
            this,
            trajLogger
        );
    }
    public AutoFactory createAutoFactory() {
        //return createAutoFactory((sample, isStart) -> {});
        return new AutoFactory(
            () -> this.getState().Pose,
            this::resetPose,
            this::followPath,
            false,
            this
        );
    }

    public AutoFactory createAutoFactoryBlue() {
        //return createAutoFactory((sample, isStart) -> {});
        return new AutoFactory(
            () -> this.getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this
        );
    }

    public SwerveSample followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        //var pose = getFrontLLPose();
        Pose2d pose = this.getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
        return null;
    }


    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        //vision.updateLimelightYaw(this); - update yaw
        //vision.updatePoseEstimatorMT2(); - update vision

        //updateLimelightYaw();
        //updatePosesEstimatorMT2();
        m_poseEstimator.update(getHeading(), getModulePositions());
        
        SmartDashboard.putNumber("Postiton X", getPose().getX());
        SmartDashboard.putNumber("Postiton Y", getPose().getY());
        SmartDashboard.putNumber("Rotation", getPose().getRotation().getDegrees());

        SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight-cg"));
         Pose2d pose = this.getState().Pose;
         SmartDashboard.putString("Pose", pose.toString());
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }
    /*public ChassisSpeeds getFieldVelocity() {
        return m_.getFieldVelocity();  //Cory stopped typing, fix sunday
      }*/
    /*public LinearVelocity getVelocityMagnitude() {
    ChassisSpeeds cs = getFieldVelocity();
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }*/

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }
    public void updateLimelightYaw(){
        double[] stddevs = NetworkTableInstance.getDefault().getTable("limelight-cg").getEntry("stddevs").getDoubleArray(new double[12]);
        
        if (stddevs[5] < 1.5){
            LimelightHelpers.SetRobotOrientation("limelight-cg",
                LimelightHelpers.getBotPose2d_wpiBlue("limelight-cg").getRotation().getDegrees(), 0, 0, 0, 0, 0);
        }else{
            LimelightHelpers.SetRobotOrientation("limelight-cg", 
                getPose().getRotation().getDegrees(),0,0,0,0,0);
        }

    }

    public void updatePosesEstimatorMT2() {

        double maxta = 0;
        String camera = null;
        PoseEstimate mt2 = new PoseEstimate();
        String[] limelights = {"limelight-cg"};
        for (String limelight: limelights) {
          LimelightHelpers.PoseEstimate megaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
          
          if (megaTag2Pose != null) {
            if(megaTag2Pose.tagCount > 0)
            {
              //we have a tag!
              //if the TA is larger than the other camera
              if(LimelightHelpers.getTA(limelight) > maxta)
              {
                maxta = LimelightHelpers.getTA(limelight);
                mt2  = megaTag2Pose;
                camera = limelight;
              }
    
            }
          }
    
        }
        if (camera != null) {
          addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
          SmartDashboard.putBoolean("limelightTV", true);
        }
        else {
          SmartDashboard.putBoolean("limelightTV", false);
        }
      }

    public Pose2d getPose(){
        return m_poseEstimator.getEstimatedPosition();
    }
    public void visionPose(Pose2d pose, double timestamp){
        m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void resetPoseEstimator(Pose2d pose){
        m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
    }
    public double getTX() {
        return m_limelight.getEntry("tx").getDouble(0.0);
      }
      public Pose2d getPoseNT() {
        return (Pose2d) NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).getEntry(getFrontLLPose(), (PubSubOption)null);
      }
      public double getTY() {
        return m_limelight.getEntry("ty").getDouble(0.0);
      }
      public double getTXFront() {
        return m_limelight.getEntry("tx").getDouble(0.);
      }
      public double getTYFront() {
        return m_limelight.getEntry("ty").getDouble(0.);
      }
      public boolean getTVFront() {
        return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public boolean getTV() {
        return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public boolean getTVRear() {
        return m_limelight.getEntry("tv").getDouble(0.0) == 1.0;
      }
      public double getTZ() {
        return m_limelight.getEntry("ty").getDouble(0.0);
      }
      public Pose2d getPoseLL() {
        var array = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
        return pose;
        // double[] poseArray = {pose.getX(), pose.getY(), ((pose.getRotation().getDegrees())/360)+(pose.getRotation().getDegrees()%360)};
        // table.getEntry("RobotPose").setDoubleArray(poseArray);
        // SmartDashboard.putNumberArray("Raw Pose", result);
      }
      public Pose2d getRearLLPose() {
        var array = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]));
        return pose;
      }
      public Pose2d getFrontLLPose() {
        var array = m_limelight.getEntry("botpose_wpired").getDoubleArray(new double[]{});
        double[] result = {array[0], array[1], array[5]};
        Pose2d pose = new Pose2d(result[0], result[1], new Rotation2d(result[2]*(Math.PI/180)));
        return pose;
      }
      public SwerveModulePosition[] getModulePositions() {
        return getState().ModulePositions;
      }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public void drive(RobotCentric withRotationalRate) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'drive'");
    }
}
