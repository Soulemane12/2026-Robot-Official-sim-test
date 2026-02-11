package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.FuelSim;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterMotor;
    private boolean m_isRunning = false;
    private FuelSim m_fuelSim;
    private Supplier<ChassisSpeeds> m_robotSpeedsSupplier;

    // Full speed (100% duty cycle)
    private static final double SHOOTER_SPEED = 1.0;

    // Distance-based shooting parameters (can be tuned)
    private static final double MIN_VELOCITY = 6.8;   // m/s for close shots
    private static final double MAX_VELOCITY = 10.8;  // m/s for far shots
    private static final double MIN_ANGLE = 58.0;     // degrees for close shots
    private static final double MAX_ANGLE = 68.0;     // degrees for far shots
    private static final double CLOSE_DISTANCE = 2.0; // meters
    private static final double FAR_DISTANCE = 6.0;  // meters
    private static final double DEFAULT_DISTANCE = 3.2; // meters when no valid estimate yet
    private static final double DISTANCE_FILTER_ALPHA = 0.35;

    // Shoot-on-the-move compensation tuning
    private static final double MIN_FORWARD_COMPONENT = 0.25; // m/s
    private static final double VELOCITY_HEADROOM = 2.0;      // m/s above max map velocity
    private static final double MAX_TURRET_COMPENSATION_DEG = 25.0;

    private frc.robot.subsystems.VisionSubsystem m_visionSubsystem;
    private double m_lastDistanceMeters = DEFAULT_DISTANCE;
    private boolean m_hasDistanceEstimate = false;

    public ShooterSubsystem(int motorCanId) {
        m_shooterMotor = new TalonFX(motorCanId);

        // Configure motor
        m_shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        // Optional: Set current limit to protect motor
        // var currentLimits = new CurrentLimitsConfigs();
        // currentLimits.StatorCurrentLimit = 80;
        // currentLimits.StatorCurrentLimitEnable = true;
        // m_shooterMotor.getConfigurator().apply(currentLimits);
    }

    @Override
    public void periodic() {
        // Publish shooter state to SmartDashboard
        SmartDashboard.putBoolean("Shooter/IsRunning", m_isRunning);
        SmartDashboard.putNumber("Shooter/MotorOutput", m_shooterMotor.get());
        SmartDashboard.putNumber("Shooter/Velocity", m_shooterMotor.getVelocity().getValueAsDouble());
    }

    /**
     * Toggle the shooter on/off
     */
    public void toggle() {
        if (m_isRunning) {
            stop();
        } else {
            start();
        }
    }

    /**
     * Start the shooter at full speed
     */
    public void start() {
        m_shooterMotor.set(SHOOTER_SPEED);
        m_isRunning = true;
    }

    /**
     * Stop the shooter
     */
    public void stop() {
        m_shooterMotor.set(0);
        m_isRunning = false;
    }

    /**
     * Check if shooter is running
     */
    public boolean isRunning() {
        return m_isRunning;
    }

    /**
     * Sets the FuelSim instance for launching projectiles in simulation
     */
    public void setFuelSim(FuelSim fuelSim) {
        m_fuelSim = fuelSim;
    }

    /**
     * Sets the robot speeds supplier for shoot-while-moving compensation
     */
    public void setRobotSpeedsSupplier(Supplier<ChassisSpeeds> speedsSupplier) {
        m_robotSpeedsSupplier = speedsSupplier;
    }

    /**
     * Sets the vision subsystem for distance-based shooting
     */
    public void setVisionSubsystem(frc.robot.subsystems.VisionSubsystem visionSubsystem) {
        m_visionSubsystem = visionSubsystem;
    }

    /**
     * Launches a fuel projectile in simulation with distance-based parameters and shoot-while-moving compensation
     */
    public void shootBall() {
        if (m_fuelSim != null && RobotBase.isSimulation()) {
            double distance = getShotDistanceMeters();

            // Calculate shooting parameters based on distance
            double distanceRatio = MathUtil.clamp(
                (distance - CLOSE_DISTANCE) / (FAR_DISTANCE - CLOSE_DISTANCE),
                0.0,
                1.0
            );

            double mappedLaunchVelocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * distanceRatio;
            double launchAngleDeg = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * distanceRatio;
            double launchVelocity = mappedLaunchVelocity;

            // Calculate launch/turret compensation for shooting while moving.
            // Compensates both forward/backward velocity (vx) and strafe velocity (vy).
            double turretYawDeg = 0.0; // Default: shoot straight ahead

            if (m_robotSpeedsSupplier != null) {
                ChassisSpeeds robotSpeeds = m_robotSpeedsSupplier.get();

                double launchAngleRad = Math.toRadians(launchAngleDeg);
                double cosAngle = Math.max(0.05, Math.cos(launchAngleRad));
                double desiredHorizontalSpeed = mappedLaunchVelocity * cosAngle;

                double requiredForwardSpeed = Math.max(
                    MIN_FORWARD_COMPONENT,
                    desiredHorizontalSpeed - robotSpeeds.vxMetersPerSecond
                );
                double requiredStrafeSpeed = -robotSpeeds.vyMetersPerSecond;
                double compensatedHorizontalSpeed = Math.hypot(requiredForwardSpeed, requiredStrafeSpeed);

                double maxCompensatedVelocity = MAX_VELOCITY + VELOCITY_HEADROOM;
                launchVelocity = MathUtil.clamp(
                    compensatedHorizontalSpeed / cosAngle,
                    MIN_VELOCITY,
                    maxCompensatedVelocity
                );
                turretYawDeg = Math.toDegrees(Math.atan2(requiredStrafeSpeed, requiredForwardSpeed));
                turretYawDeg = MathUtil.clamp(
                    turretYawDeg,
                    -MAX_TURRET_COMPENSATION_DEG,
                    MAX_TURRET_COMPENSATION_DEG
                );

                SmartDashboard.putNumber("Shooter/RobotVX", robotSpeeds.vxMetersPerSecond);
                SmartDashboard.putNumber("Shooter/RobotVY", robotSpeeds.vyMetersPerSecond);
                SmartDashboard.putNumber("Shooter/DesiredHorizontalSpeed", desiredHorizontalSpeed);
                SmartDashboard.putNumber("Shooter/CompensatedHorizontalSpeed", compensatedHorizontalSpeed);
            }

            SmartDashboard.putNumber("Shooter/Distance", distance);
            SmartDashboard.putNumber("Shooter/MappedLaunchVelocity", mappedLaunchVelocity);
            SmartDashboard.putNumber("Shooter/LaunchVelocity", launchVelocity);
            SmartDashboard.putNumber("Shooter/LaunchAngle", launchAngleDeg);
            SmartDashboard.putNumber("Shooter/TurretCompensation", turretYawDeg);

            m_fuelSim.launchFuel(
                MetersPerSecond.of(launchVelocity),
                Degrees.of(launchAngleDeg),
                Degrees.of(turretYawDeg),  // Apply compensation angle
                Meters.of(0.5)
            );

            System.out.println(String.format(
                "[Shooter] Fuel launched: dist=%.2fm, vel=%.1fm/s (mapped %.1f), angle=%.1f°, turret=%.1f°",
                distance,
                launchVelocity,
                mappedLaunchVelocity,
                launchAngleDeg,
                turretYawDeg
            ));
        }
    }

    private double getShotDistanceMeters() {
        boolean hasTargetTag = false;
        boolean usingVisionDistance = false;
        double rawVisionDistance = -1.0;

        if (m_visionSubsystem != null && m_visionSubsystem.hasValidTarget()) {
            hasTargetTag = (int) m_visionSubsystem.getTagID() == Constants.VisionConstants.TARGET_APRILTAG_ID;
            if (hasTargetTag) {
                rawVisionDistance = m_visionSubsystem.getDistanceToAprilTag();
                if (rawVisionDistance > 0.1 && rawVisionDistance < 12.0) {
                    if (m_hasDistanceEstimate) {
                        m_lastDistanceMeters += DISTANCE_FILTER_ALPHA * (rawVisionDistance - m_lastDistanceMeters);
                    } else {
                        m_lastDistanceMeters = rawVisionDistance;
                        m_hasDistanceEstimate = true;
                    }
                    usingVisionDistance = true;
                }
            }
        }

        if (!m_hasDistanceEstimate) {
            m_lastDistanceMeters = DEFAULT_DISTANCE;
            m_hasDistanceEstimate = true;
        }

        double clampedDistance = MathUtil.clamp(m_lastDistanceMeters, CLOSE_DISTANCE, FAR_DISTANCE);

        SmartDashboard.putBoolean("Shooter/HasTargetTag", hasTargetTag);
        SmartDashboard.putBoolean("Shooter/UsingVisionDistance", usingVisionDistance);
        SmartDashboard.putNumber("Shooter/RawVisionDistance", rawVisionDistance);
        SmartDashboard.putNumber("Shooter/FilteredDistance", clampedDistance);

        return clampedDistance;
    }
}
