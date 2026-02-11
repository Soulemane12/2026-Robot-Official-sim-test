package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FuelSim;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterMotor;
    private boolean m_isRunning = false;
    private FuelSim m_fuelSim;
    private Supplier<ChassisSpeeds> m_robotSpeedsSupplier;

    // Full speed (100% duty cycle)
    private static final double SHOOTER_SPEED = 1.0;

    // Shooting parameters for flight time calculation
    private static final double GRAVITY = 9.81; // m/s^2

    // Distance-based shooting parameters (can be tuned)
    private static final double MIN_VELOCITY = 7.5;  // m/s for close shots (increased power)
    private static final double MAX_VELOCITY = 11.0;  // m/s for far shots (increased power)
    private static final double MIN_ANGLE = 75;      // degrees for close shots (extremely high arc)
    private static final double MAX_ANGLE = 85;      // degrees for far shots (extremely high arc - almost vertical)
    private static final double CLOSE_DISTANCE = 2.0; // meters
    private static final double FAR_DISTANCE = 6.0;  // meters

    private frc.robot.subsystems.VisionSubsystem m_visionSubsystem;

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
            // Get distance to target from vision subsystem
            double distance = FAR_DISTANCE; // Default to far distance
            if (m_visionSubsystem != null && m_visionSubsystem.hasValidTarget()) {
                distance = m_visionSubsystem.getDistanceToAprilTag();
            }

            // Calculate shooting parameters based on distance
            // Closer targets = lower velocity, flatter angle
            // Farther targets = higher velocity, steeper angle
            double distanceRatio = Math.max(0, Math.min(1,
                (distance - CLOSE_DISTANCE) / (FAR_DISTANCE - CLOSE_DISTANCE)));

            double launchVelocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * distanceRatio;
            double launchAngleDeg = MIN_ANGLE + (MAX_ANGLE - MIN_ANGLE) * distanceRatio;

            // Calculate turret angle compensation for shooting while moving
            double turretYawDeg = 0.0; // Default: shoot straight ahead

            if (m_robotSpeedsSupplier != null) {
                ChassisSpeeds robotSpeeds = m_robotSpeedsSupplier.get();

                // Calculate approximate flight time
                // For a projectile: t = 2 * v_z / g where v_z = v * sin(angle)
                double launchAngleRad = Math.toRadians(launchAngleDeg);
                double verticalVelocity = launchVelocity * Math.sin(launchAngleRad);
                double flightTime = 2.0 * verticalVelocity / GRAVITY;

                // Calculate lateral offset due to robot motion
                // From robot's perspective, target moves with velocity (-vx, -vy)
                // We care about the strafe component (vy) for turret compensation
                // FIXED: Use positive offset (robot moving left = aim left)
                double lateralOffset = flightTime * robotSpeeds.vyMetersPerSecond;

                // Calculate horizontal distance the projectile travels
                double horizontalVelocity = launchVelocity * Math.cos(launchAngleRad);
                double horizontalDistance = horizontalVelocity * flightTime;

                // Calculate turret angle needed to compensate
                // tan(angle) = lateral_offset / horizontal_distance
                if (horizontalDistance > 0.1) { // Avoid division by zero
                    turretYawDeg = Math.toDegrees(Math.atan2(lateralOffset, horizontalDistance));
                }

                SmartDashboard.putNumber("Shooter/RobotVY", robotSpeeds.vyMetersPerSecond);
                SmartDashboard.putNumber("Shooter/FlightTime", flightTime);
                SmartDashboard.putNumber("Shooter/LateralOffset", lateralOffset);
                SmartDashboard.putNumber("Shooter/TurretCompensation", turretYawDeg);
            }

            SmartDashboard.putNumber("Shooter/Distance", distance);
            SmartDashboard.putNumber("Shooter/LaunchVelocity", launchVelocity);
            SmartDashboard.putNumber("Shooter/LaunchAngle", launchAngleDeg);

            m_fuelSim.launchFuel(
                MetersPerSecond.of(launchVelocity),
                Degrees.of(launchAngleDeg),
                Degrees.of(turretYawDeg),  // Apply compensation angle
                Meters.of(0.5)
            );

            System.out.println(String.format("[Shooter] Fuel launched: dist=%.2fm, vel=%.1fm/s, angle=%.1f°, turret=%.1f°",
                distance, launchVelocity, launchAngleDeg, turretYawDeg));
        }
    }
}
