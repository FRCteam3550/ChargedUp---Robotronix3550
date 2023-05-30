package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.*;
import com.swervedrivespecialties.swervelib.ctre.CANCoderAbsoluteEncoder;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveConfiguration;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerConfiguration;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PilotShuffleboardLayout;
import frc.robot.commands.SelfReplacingCommand;
import frc.robot.subsystems.drivetrain.ChargedUpField;
import frc.robot.subsystems.drivetrain.Odometry;
import frc.robot.subsystems.drivetrain.SpeedFilter;
import frc.robot.subsystems.drivetrain.SpeedSetPoint;
import frc.robot.subsystems.drivetrain.TrajectoryFollowing;
import frc.robot.utils.Navx;
import frc.robot.utils.Pose;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.ctre.Falcon500DriveController;
import com.swervedrivespecialties.swervelib.ctre.Falcon500SteerController;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DrivetrainSubsystem extends SubsystemBase {
    private static enum SpeedMode {
        Pct,
        MetersPerSecond
    }
    
    private static final double DEADBAND = .09;
    private static final double VELOCITY_RANGE_LIMIT = .3;
    // valeur 'usine': 0.3152m
    // ancienne circonference des roues = .3085m (Finger Lakes)
    // nouvelle circonference des roues = .3225m (Changee pour Trois-Rivieres)
    private static final double DISTANCE_BY_WHEEL_ROTATION_M = .315;

    private static final double FRONT_SIDE_M = .415;
    private static final double RIGHT_SIDE_M = .596;
    private static final double BACK_SIDE_M = .420;
    private static final double LEFT_SIDE_M = .594;
    private static final double ROBOT_CENTER_TO_FRONT_BUMPER_M = 0.46;
 //   private static final double CLAW_CENTER_TO_FRONT_BUMPER_M = 0; // A changer

    private static final double FRONT_RIGHT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double FRONT_LEFT_MODULE_X_M = FRONT_SIDE_M / 2;
    private static final double FRONT_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;

    private static final double BACK_RIGHT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_RIGHT_MODULE_Y_M = -RIGHT_SIDE_M / 2;

    private static final double BACK_LEFT_MODULE_X_M = -BACK_SIDE_M / 2;
    private static final double BACK_LEFT_MODULE_Y_M = LEFT_SIDE_M / 2;
    private static final int CAN_CODER_READING_UPDATE_PERIOD_MS = 100;

    private static final double K_P_BACKWARD = 1.4; // ok
    private static final double K_I_BACKWARD = 0; 
    private static final double K_D_BACKWARD = 0; 

    private static final double K_P_ROTATION = 0.04; // OK
    private static final double K_I_ROTATION = 0; 
    private static final double K_D_ROTATION = 0; 

    private static final double TURN_TOLERANCE_DEG = 2;
    private static final double TURN_RATE_TOLERANCE_DEG_PER_S = 3;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = TEMP_MAX_SPEED_MS /
        Math.hypot(FRONT_SIDE_M, RIGHT_SIDE_M);

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(FRONT_LEFT_MODULE_X_M, FRONT_LEFT_MODULE_Y_M),
        // Front right
        new Translation2d(FRONT_RIGHT_MODULE_X_M, FRONT_RIGHT_MODULE_Y_M),
        // Back left
        new Translation2d(BACK_LEFT_MODULE_X_M, BACK_LEFT_MODULE_Y_M),
        // Back right
        new Translation2d(BACK_RIGHT_MODULE_X_M, BACK_RIGHT_MODULE_Y_M)
    );

    private static final ShuffleboardTab SHUFFLEBOARD_TAB = Shuffleboard.getTab("Drivetrain");  

    private static final ChassisSpeeds STOP_SPEEDS = new ChassisSpeeds(0, 0, 0);

    private static final SwerveModuleState[] STOP_STATES = KINEMATICS.toSwerveModuleStates(STOP_SPEEDS);

    private final ChargedUpField m_field = new ChargedUpField();

    private final AHRS m_navx = Navx.newReadyNavx(); // NavX connected over MXP

    private final SwerveModule[] m_modules = {
            createModule(
                "Front Left Module",
                0,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_ENCODER_ID,
                FRONT_LEFT_MODULE_STEER_ALIGN_ANGLE),

            createModule(
                "Front Right Module",
                2,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
                FRONT_RIGHT_MODULE_STEER_ALIGN_ANGLE),

            createModule(
                "Back Left Module",
                4,
                BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_ENCODER_ID,
                BACK_LEFT_MODULE_STEER_ALIGN_ANGLE),

            createModule(
                "Back Right Module",
                6,
                BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_ENCODER_ID,
                BACK_RIGHT_MODULE_STEER_ALIGN_ANGLE),
    };

    private final Odometry m_odometry = new Odometry(KINEMATICS, getGyroscopeRotation(), getModulePositions());
    private final SpeedFilter m_speedFilter = new SpeedFilter()
        .withDeadband(DEADBAND)
        .withRangeLimits(1, VELOCITY_RANGE_LIMIT)
        .withConversionRatios(TEMP_MAX_SPEED_MS, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        .withTranslationInputSpeedLimit(1.5);


    private final TrajectoryFollowing m_trajectoryFollowing =  new TrajectoryFollowing(
        KINEMATICS,
        m_odometry::getPoseM, 
        (states) -> m_states = states, 
        this,
        m_field::displayTrajectory
    );
    private final XboxController m_gamepad;
    private SpeedMode m_speedMode = SpeedMode.Pct;

    private SwerveModuleState[] m_states = STOP_STATES;

    public DrivetrainSubsystem(XboxController gamepad) {
        m_gamepad = gamepad;
        setDefaultCommand(drive());
        //setDefaultCommand(driveFrontRightVelManually());
        SmartDashboard.putData(this);
        PilotShuffleboardLayout.ODO_LAYOUT.addNumber("pitch", () -> getPitchAngleDegrees());
        PilotShuffleboardLayout.ODO_LAYOUT.add(resetOdometry());
    }

    private static SwerveModule createModule(
        String name, 
        int shuffleBoardPosition,
        int driveMotorPort,
        int steerMotorPort,
        int steerEncoderPort,
        DiscreetAngle steerAlignAngle
    ) {
        var layout = SHUFFLEBOARD_TAB.getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(shuffleBoardPosition, 0);

        var gearRatio = SdsGearRatios.MK4_L1.withWheelCircumference(DISTANCE_BY_WHEEL_ROTATION_M);

        var driveConfiguration = new Falcon500DriveConfiguration(driveMotorPort)
          .withCurrentLimit(30)
          .withPidConstants(0.06, 0.035, 0, 0.0001);
//        .withPidConstants(Double.NaN, -4, 0, 0);
        var driveController = new Falcon500DriveController(driveConfiguration, gearRatio);

        var absoluteEncoder = new CANCoderAbsoluteEncoder(steerEncoderPort, steerAlignAngle, CAN_CODER_READING_UPDATE_PERIOD_MS);
        var steerConfiguration = new Falcon500SteerConfiguration(steerMotorPort, absoluteEncoder)
            .withPidConstants(0.75, 0, 2);
        var steerController = new Falcon500SteerController(steerConfiguration, gearRatio);

        var result = new DefaultSwerveModule(driveController, steerController);
        result.configureShuffleboardContainer(layout);
        
        return result;
    }

    private double getPitchAngleDegrees() {
        return m_navx.getRoll();
    }

    private Command setOdometryPose(Pose2d pose) {
        return runOnce(() ->
            m_odometry.setPose(getGyroscopeRotation(), getModulePositions(), pose)
        );
    }

    private static double capSpeed(double speedPct, double maxSpeedPct) {
        if (Math.abs(speedPct) > maxSpeedPct) {
            return Math.copySign(maxSpeedPct, speedPct);
        }
        return speedPct;
    }

    private void setXAxisMotorOutput(double speedPct, double maxSpeedPct) {
        final var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            capSpeed(speedPct, maxSpeedPct) * TEMP_MAX_SPEED_MS,
            0,
            0,
            m_odometry.getPoseM().getRotation()
        );
        m_states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    private void setRotateMotorOutput(double speedPct) {
        final var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            capSpeed(speedPct, 0.5) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            m_odometry.getPoseM().getRotation()
        );
        m_states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    }

    public Command testPIDSpeed() {
        return run(() -> {
            setXAxisMotorOutput(0.2, 0.2);
        })
            .beforeStarting(() -> m_speedMode = SpeedMode.MetersPerSecond)
            .withTimeout(2)
            .andThen(() -> m_speedMode = SpeedMode.Pct);
    }
    

    private Command selfReplacing(Supplier<Command> makeReplacement) {
        return new SelfReplacingCommand(makeReplacement, this);
    }

    private Command goToViaPIDAlongXAxisPIDSpeed(Pose2d destinationM, double maxSpeed) {
        final var PID_CONTROLLER = new PIDController(K_P_BACKWARD, K_I_BACKWARD, K_D_BACKWARD);
        PID_CONTROLLER.setTolerance(0.015, 0.04);
        var returnX = new DoubleSupplier() {
            double maxX = 0.0;

            @Override
            public double getAsDouble() {
                var x = m_odometry.getPoseM().getX();
                if (x > maxX) {
                    maxX = x;
                }
                return x;
            }
            
        };

        return new PIDCommand(
            PID_CONTROLLER,
            returnX,
            destinationM::getX,
            (pct) -> setXAxisMotorOutput(pct, maxSpeed),
            this
        )
        .beforeStarting(() -> m_speedMode = SpeedMode.MetersPerSecond)
        .until(() -> PID_CONTROLLER.atSetpoint())
        .andThen(() -> {
            m_speedMode = SpeedMode.Pct;
            stopMotor();

            SmartDashboard.putNumber("Max X", returnX.maxX);
            SmartDashboard.putNumber("End odo X", m_odometry.getPoseM().getX());
        });
    }

    private Command goAwayFromGridUntilXPIDSpeedNoStop(double destinationXM, double speed) {
        return run(() -> setXAxisMotorOutput(speed, speed))
            .beforeStarting(() -> m_speedMode = SpeedMode.MetersPerSecond)
            .until(() -> 
                (m_field.gridRobotXDirection() < 0 && m_odometry.getPoseM().getX() > destinationXM) ||
                (m_field.gridRobotXDirection() > 0 && m_odometry.getPoseM().getX() < destinationXM)
            )
            .andThen(() ->  m_speedMode = SpeedMode.Pct);
    }

    private Command goToViaPIDAlongXAxis(Pose2d destinationM, double maxSpeed) {
        final var PID_CONTROLLER = new PIDController(K_P_BACKWARD, K_I_BACKWARD, K_D_BACKWARD);
        PID_CONTROLLER.setTolerance(0.02, 0.05);

        return new PIDCommand(
            PID_CONTROLLER,
            () -> m_odometry.getPoseM().getX(),
            destinationM::getX,
            (pct) -> setXAxisMotorOutput(pct, maxSpeed),
            this
        )
        .until(PID_CONTROLLER::atSetpoint);
    }

    private Command rotate(Rotation2d targetFieldRelativeAngle) {
        final var PID_CONTROLLER = new PIDController(K_P_ROTATION, K_I_ROTATION, K_D_ROTATION);
        // m_odometry.getPoseM().getRotation().getDegrees() retourne un angle entre 0 et 360 car
        // l'angle de l'odométrie provient du gyro, et donc de m_navx.getFusedHeading().
        PID_CONTROLLER.enableContinuousInput(0, 360);
        PID_CONTROLLER.setTolerance(TURN_TOLERANCE_DEG, TURN_RATE_TOLERANCE_DEG_PER_S);

        return new PIDCommand(
            PID_CONTROLLER,
            () -> m_odometry.getPoseM().getRotation().getDegrees(),
            targetFieldRelativeAngle::getDegrees,
            this::setRotateMotorOutput,
            this
        )
        .until(PID_CONTROLLER::atSetpoint);
    }

    private Command backwardUntilInclined() {
        final var SPEED_PCT = 0.06;
        final var NO_LIMIT_PCT = 1.0;
        final var INCLINED_THRESHOLD_DEGREES = -2;

        return run(() -> setXAxisMotorOutput(-SPEED_PCT * m_field.gridRobotXDirection(), NO_LIMIT_PCT))
            .beforeStarting(() -> m_speedMode = SpeedMode.MetersPerSecond)
            .until(() -> getPitchAngleDegrees() < INCLINED_THRESHOLD_DEGREES)
            .andThen(run(() -> {
                stopMotor();
                m_speedMode = SpeedMode.Pct;
            }).withTimeout(0.3));
    }

    private Command forwardUntilInclined() {
        final var SPEED_PCT = 0.11;
        final var NO_LIMIT_PCT = 1.0;
        final var INCLINED_THRESHOLD_DEGREES = 3;

        return run(() -> setXAxisMotorOutput(SPEED_PCT *-m_field.gridRobotXDirection(), NO_LIMIT_PCT))
            .beforeStarting(() -> m_speedMode = SpeedMode.MetersPerSecond)
            .until(() -> getPitchAngleDegrees() > INCLINED_THRESHOLD_DEGREES)
            .andThen(() -> {
                stopMotor();
                m_speedMode = SpeedMode.Pct;
            });
    }

    public Command exitCommunityFromNode(int nodeIndex) {
        final var SPEED_PCT = 0.3;
        final var NODE_START_POSE_M = m_field.gridNodePose(nodeIndex, ROBOT_CENTER_TO_FRONT_BUMPER_M);
        // var destinationPose = new Pose2d(
        //     initialPose.getX() + 2,
        //     initialPose.getY(),
        //     initialPose.getRotation()
        // );
        final var EXIT_POSE_M = Pose.withNewX(NODE_START_POSE_M, m_field.exitCommunityXM());

        return setOdometryPose(NODE_START_POSE_M)
            .andThen(goToViaPIDAlongXAxis(EXIT_POSE_M, SPEED_PCT))
            .withName("exitCom");
    }

    public Command exitCommunityFromNodeWithTrajectoryFollowing(int nodeIndex) {
        final var NODE_START_POSE_M = m_field.gridNodePose(nodeIndex, ROBOT_CENTER_TO_FRONT_BUMPER_M);
        final var EXIT_POSE_M = Pose.withNewX(NODE_START_POSE_M, m_field.exitCommunityXM());

        return setOdometryPose(NODE_START_POSE_M)
            .andThen(m_trajectoryFollowing.goToLocation(NODE_START_POSE_M, EXIT_POSE_M, m_field.towardOppositeAlliance()))
            .withName("exitComWithTrajectoryFollowing");
    }

    public Command halfTurn() {
        final var HALF_TURN_DEGREES = 180;

        return selfReplacing(() -> {
            var targetAngle = m_odometry
                .getPoseM()
                .getRotation()
                .plus(Rotation2d.fromDegrees(HALF_TURN_DEGREES));

            return rotate(targetAngle);
        }).withName("halfTurn");
    }

    public Command engageChargingStationFromNodeAndBalance() {
        final var NODE_START_POSE_M = m_field.gridNodePose(3, ROBOT_CENTER_TO_FRONT_BUMPER_M);

        final var CHARGE_STATION_POSE_M = Pose.withNewX(NODE_START_POSE_M, m_field.chargeStationCenterM().getX() - .1 * m_field.gridRobotXDirection());
        final var REACH_STATION_SPEED_PCT = 0.18;

        final var TO_BALANCED_DISTANCE_M = 0.79;
        final var REACH_BALANCE_SPEED_PCT = 0.45;


        return setOdometryPose(NODE_START_POSE_M)
            .andThen(goToViaPIDAlongXAxisPIDSpeed(CHARGE_STATION_POSE_M, REACH_STATION_SPEED_PCT))
            .andThen(backwardUntilInclined())
            .andThen(selfReplacing(() -> {
                var fromPose = m_odometry.getPoseM();
                var toPose = Pose.withNewX(fromPose, fromPose.getX() + TO_BALANCED_DISTANCE_M * m_field.gridRobotXDirection());
                SmartDashboard.putNumber("Start X", fromPose.getX());
                SmartDashboard.putNumber("End X", toPose.getX());

                return goToViaPIDAlongXAxisPIDSpeed(toPose, REACH_BALANCE_SPEED_PCT);
            }))
            .withName("engageChargingStationFromNodeAndBalance");
    }

    // Fonctionne, mais trop lent (20s)
    // public Command exitCommunityAndEngagedOption1() {
        
    //     return new SelfReplacingCommand(() -> {
    //         var initialPose = m_field.gridNodePose(3, ROBOT_CENTER_TO_FRONT_BUMPER_M);
    //         var destinationPose = Pose.withNewX(initialPose, m_field.exitCommunityXM() + 0.2 * m_field.gridRobotXDirection());
    //         //var acceleration = Pose.withNewX(initialPose, destinationPose - 0.5);

    //         setOdometryPose(initialPose);

    //         return goToViaPIDAlongXAxisPIDSpeed(destinationPose, 0.18);
    //     }, this)
    //     .andThen(forwardUntilInclined())
    //     .andThen(new SelfReplacingCommand(() -> {
    //         var fromPose = getOdometryPoseM();
    //         var toPose = Pose.withNewX(fromPose, fromPose.getX() + 0.50 * -m_field.gridRobotXDirection());

    //         return goToViaPIDAlongXAxisPIDSpeed(toPose, 0.30);
    //     }, this));
    // }

    // Fonctionne, mais encore trop lent (18.5s)
    public Command exitCommunityAndEngagedOption2() {
        final var NODE_START_POSE_M = m_field.gridNodePose(3, ROBOT_CENTER_TO_FRONT_BUMPER_M);

        final var CHARGE_STATION_POSE_M = Pose.withNewX(NODE_START_POSE_M, m_field.chargeStationCenterM());
        final var REACH_STATION_CENTER_SPEED_PCT = 0.2;

        final var EXIT_POSE_M = Pose.withNewX(CHARGE_STATION_POSE_M, m_field.exitCommunityXM() + 0 * m_field.gridRobotXDirection());
        final var REACH_EXIT_SPEED_PCT = 0.20;

        final var BACK_TO_STATION_DISTANCE_M = 0.5;
        final var BACK_TO_STATION_SPEED_PCT = 0.35;

        final var TO_BALANCED_DISTANCE_M = 0.7;
        final var REACH_BALANCE_SPEED_PCT = 0.35;

        return setOdometryPose(NODE_START_POSE_M)
            .andThen(goAwayFromGridUntilXPIDSpeedNoStop(m_field.chargeStationCenterM().getX(), REACH_STATION_CENTER_SPEED_PCT)) // Tenter d'augmenter
            .andThen(goToViaPIDAlongXAxisPIDSpeed(EXIT_POSE_M,REACH_EXIT_SPEED_PCT))
            .andThen(selfReplacing(() -> {
                var fromPose = m_odometry.getPoseM();
                var toPose = Pose.withNewX(fromPose, fromPose.getX() + BACK_TO_STATION_DISTANCE_M * m_field.gridRobotXDirection());

                return goToViaPIDAlongXAxisPIDSpeed(toPose, BACK_TO_STATION_SPEED_PCT); // Tenter d'augmenter
            }))
            .andThen(forwardUntilInclined())
            .andThen(selfReplacing(() -> {
                var fromPose = m_odometry.getPoseM();
                var toPose = Pose.withNewX(fromPose, fromPose.getX() + TO_BALANCED_DISTANCE_M * -m_field.gridRobotXDirection());

                return goToViaPIDAlongXAxisPIDSpeed(toPose, REACH_BALANCE_SPEED_PCT);
            }))
            .withName("exitCommunityAndEngagedOption2");
    }

    public Command testTrajectory() {
        return selfReplacing(() -> {
            final var TRAJECTORY_TEST_M = 1;
            var fromPose = m_odometry.getPoseM();
            var toPose = Pose.withNewX(fromPose, fromPose.getX() - TRAJECTORY_TEST_M * m_field.gridRobotXDirection());

            return m_trajectoryFollowing.goToLocation(fromPose, toPose, m_field.towardSelfAlliance());
        });
    }

    public Command trajectory2PiecesAuto() {
        var halfTurn = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180));
        var fromPose = m_field.gridNodePose(8, ROBOT_CENTER_TO_FRONT_BUMPER_M);
        var toPose = m_field.initialPiecePoseM(3, 0.95);                                                              
        return resetOdometry(fromPose)
            .andThen(m_trajectoryFollowing.goToLocation(fromPose.transformBy(halfTurn), toPose, m_field.towardSelfAlliance()))
            .withName("trajectory2PiecesAuto");
    }

    public Command trajectory2PiecesAutoWithHalfTurn() {
        var halfTurn = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180));
        var fromPose = m_field.gridNodePose(8, ROBOT_CENTER_TO_FRONT_BUMPER_M);
        var toPose = m_field.initialPiecePoseM(3, 0.95).transformBy(halfTurn);                                                              
        return resetOdometry(fromPose)
            .andThen(m_trajectoryFollowing.goToLocation(fromPose.transformBy(halfTurn), toPose, m_field.towardOppositeAlliance()))
            .withName("trajectory2PiecesAutoWithHalfTurn");
    }

    public Command grabCubeFromNode8() {
        var trajectory = m_field.grabCubeFromNode8Trajectory();

        return resetOdometry(trajectory.getInitialPose())
            .andThen(m_trajectoryFollowing.follow(trajectory, m_field.towardSelfAlliance()))
            .withName("grabCubeFromNode8");
    }

    public Command bringCubeToNode7() {
        var trajectory = m_field.bringCubeToNode7Trajectory();

        return m_trajectoryFollowing.follow(trajectory, m_field.towardSelfAlliance())
            .withName("bringCubeToNode7");
    }
    

    public Command engageFromNode7() {
        var trajectory = m_field.engageFromNode7Trajectory();

        return m_trajectoryFollowing.follow(trajectory, m_field.towardSelfAlliance())
            .withName("engageFromNode7");
    }
    
    public Command testRedGrabCubeTrajectory() {
        var trajectory = ChargedUpField.TEST_GRAB_CUBE;

        return resetOdometry(trajectory.getInitialPose())
            .andThen(m_trajectoryFollowing.follow(trajectory, m_field.towardSelfAlliance()))
            .withName("testRedGrabCubeTrajectory");
    }

    /**
     * Retourne l'angle du Gyromètre dans l'intervalle [0, 360[
     */
    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(360 - m_navx.getFusedHeading());
    }

    public Command switchToPreciseMode () {
        return runOnce(() -> {
            m_speedFilter.switchToPreciseMode();
        })
        .withName("switchToPreciseMode");
    }

    public Command switchToFastMode () {
        return runOnce(() -> {
            m_speedFilter.switchToFastMode();
        })
        .withName("switchToFastMode");
    }

    private Command resetOdometry(Pose2d poseM) {
        return Commands.runOnce(() -> m_odometry.setPose(getGyroscopeRotation(), getModulePositions(), poseM));
    }

    public CommandBase resetOdometry() {
        return Commands
            .runOnce(() -> m_odometry.setPose(getGyroscopeRotation(), getModulePositions(), Odometry.INITIAL_POSE))
            .withName("Reset");
    }

    private void stopMotor() {
        m_states = STOP_STATES;
    }

    /**
     * Commande un seul des moteurs de la swerve avec le Joystick.
     * Sert à calibrer le PID interne de contrôle de vitesse dans le Phoenix Tuner.
     * Remplacer le default command dans le constructeur pour s'en servir.
     */
    // private Command driveFrontRightVelManually() {
    //     final var FRONT_RIGHT_MODULE_ID = 1;

    //     return run(() ->
    //         m_modules[FRONT_RIGHT_MODULE_ID].setMS(
    //             m_gamepad.getLeftX() * MAX_VELOCITY_METERS_PER_SECOND,
    //             DiscreetAngle.fromDegrees(0)
    //         )
    //     )
    //     .andThen(this::stopMotor)
    //     .withName("driveFrontRightVelManually");
    // }

    public Command switchAlliance() {
        return Commands.runOnce(() -> {
            m_field.switchColorAlliance();
        }).ignoringDisable(true);
    }
    
    public Command drive() {
        var cmd = run(() -> {
            var gamepadSpeeds = new SpeedSetPoint(
                m_gamepad.getLeftX(),
                -m_gamepad.getLeftY(),
                -m_gamepad.getRightX()
            );
            var filteredSpeeds = m_speedFilter.filter(gamepadSpeeds);
            var allianceSign = -m_field.gridRobotXDirection();
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                filteredSpeeds.ySpeed * allianceSign,
                -filteredSpeeds.xSpeed * allianceSign,
                filteredSpeeds.rotationSpeed,
                m_odometry.getPoseM().getRotation()
            );
            // var chassisSpeeds = new ChassisSpeeds(filteredSpeeds.xSpeed, filteredSpeeds.ySpeed, filteredSpeeds.rotationSpeed);
            m_states = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        }).andThen(this::stopMotor)
        .withName("drive");
        return cmd;
    }


    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] result = new SwerveModulePosition[m_modules.length];

        for (int i = 0; i < result.length; i++) {
            result[i] = new SwerveModulePosition(
                    m_modules[i].getDrivePosition(),
                    Rotation2d.fromDegrees(m_modules[i].getSteerAngle().degrees()));
        }

        return result;
    }

    @Override
    public void periodic() {
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, TEMP_MAX_SPEED_MS);

        for (int i = 0; i < m_modules.length; i++) {
            switch(m_speedMode) {
                case Pct:
                    m_modules[i].set(
                        m_states[i].speedMetersPerSecond / TEMP_MAX_SPEED_MS,
                        DiscreetAngle.fromRotation(m_states[i].angle)
                    );
                    break;
                case MetersPerSecond:
                    m_modules[i].setMS(
                        m_states[i].speedMetersPerSecond,
                        DiscreetAngle.fromRotation(m_states[i].angle)
                    );
                    break;
            }
        }

        m_odometry.update(getGyroscopeRotation(), getModulePositions());
        m_field.updateRobotPose(m_odometry.getPoseM());
    }
}
