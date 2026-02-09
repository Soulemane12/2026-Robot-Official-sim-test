package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX m_shooterMotor;
    private boolean m_isRunning = false;
    private boolean m_wasRunning = false;

    // Full speed (100% duty cycle)
    private static final double SHOOTER_SPEED = 1.0;

    // Projectile physics constants
    private static final double LAUNCH_VELOCITY = 15.0; // m/s
    private static final double LAUNCH_ANGLE = Math.toRadians(45); // 45 degrees
    private static final double GRAVITY = 9.81; // m/s^2
    private static final double PROJECTILE_LIFETIME = 3.0; // seconds

    // Field2d for visualization
    private Field2d m_field;
    private FieldObject2d m_projectilesObject;
    private List<Projectile> m_activeProjectiles;

    // Projectile class to track individual shots
    private static class Projectile {
        Translation2d initialPosition;
        double initialVelocityX;
        double initialVelocityY;
        double launchTime;
        Rotation2d launchAngle;

        Projectile(Translation2d position, Rotation2d angle, double time) {
            this.initialPosition = position;
            this.launchAngle = angle;
            this.launchTime = time;

            // Calculate velocity components
            this.initialVelocityX = LAUNCH_VELOCITY * Math.cos(LAUNCH_ANGLE) * Math.cos(angle.getRadians());
            this.initialVelocityY = LAUNCH_VELOCITY * Math.cos(LAUNCH_ANGLE) * Math.sin(angle.getRadians());
        }

        Translation2d getPosition(double currentTime) {
            double t = currentTime - launchTime;
            double x = initialPosition.getX() + initialVelocityX * t;
            double y = initialPosition.getY() + initialVelocityY * t;
            return new Translation2d(x, y);
        }

        boolean isExpired(double currentTime) {
            return (currentTime - launchTime) > PROJECTILE_LIFETIME;
        }
    }

    private java.util.function.Supplier<Pose2d> m_robotPoseSupplier;

    public ShooterSubsystem(int motorCanId) {
        m_shooterMotor = new TalonFX(motorCanId);

        // Configure motor
        m_shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        // Optional: Set current limit to protect motor
        // var currentLimits = new CurrentLimitsConfigs();
        // currentLimits.StatorCurrentLimit = 80;
        // currentLimits.StatorCurrentLimitEnable = true;
        // m_shooterMotor.getConfigurator().apply(currentLimits);

        // Initialize projectile tracking
        m_activeProjectiles = new ArrayList<>();
        m_field = new Field2d();
        m_projectilesObject = m_field.getObject("projectiles");
        SmartDashboard.putData("Shooter Field", m_field);
    }

    /**
     * Set the robot pose supplier for projectile launching position
     */
    public void setRobotPoseSupplier(java.util.function.Supplier<Pose2d> poseSupplier) {
        m_robotPoseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        // Publish shooter state to SmartDashboard
        SmartDashboard.putBoolean("Shooter/IsRunning", m_isRunning);
        SmartDashboard.putNumber("Shooter/MotorOutput", m_shooterMotor.get());
        SmartDashboard.putNumber("Shooter/Velocity", m_shooterMotor.getVelocity().getValueAsDouble());

        // Detect shooting trigger (transition from not running to running)
        if (m_isRunning && !m_wasRunning && m_robotPoseSupplier != null) {
            launchProjectile();
        }
        m_wasRunning = m_isRunning;

        // Update projectile positions
        updateProjectiles();

        // Publish projectile count
        SmartDashboard.putNumber("Shooter/ActiveProjectiles", m_activeProjectiles.size());
    }

    private void launchProjectile() {
        Pose2d robotPose = m_robotPoseSupplier.get();

        // Launch from shooter position (assume front of robot, offset forward)
        Translation2d shooterOffset = new Translation2d(0.3, 0.0); // 0.3m forward
        Translation2d launchPosition = robotPose.getTranslation().plus(
            shooterOffset.rotateBy(robotPose.getRotation())
        );

        Projectile newProjectile = new Projectile(
            launchPosition,
            robotPose.getRotation(),
            Timer.getFPGATimestamp()
        );
        m_activeProjectiles.add(newProjectile);

        System.out.println("[Shooter] Projectile launched from " + launchPosition);
    }

    private void updateProjectiles() {
        double currentTime = Timer.getFPGATimestamp();

        // Remove expired projectiles
        m_activeProjectiles.removeIf(p -> p.isExpired(currentTime));

        // Update Field2d visualization
        if (!m_activeProjectiles.isEmpty()) {
            Pose2d[] projectilePoses = new Pose2d[m_activeProjectiles.size()];
            for (int i = 0; i < m_activeProjectiles.size(); i++) {
                Translation2d pos = m_activeProjectiles.get(i).getPosition(currentTime);
                projectilePoses[i] = new Pose2d(pos, new Rotation2d());
            }
            m_projectilesObject.setPoses(projectilePoses);
        } else {
            m_projectilesObject.setPoses(); // Clear visualization
        }

        // Update robot pose on field
        if (m_robotPoseSupplier != null) {
            m_field.setRobotPose(m_robotPoseSupplier.get());
        }
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
}
