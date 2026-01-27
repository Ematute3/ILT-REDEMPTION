package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import kotlin.math.PI

/**
 * Centralized robot configuration.
 * All tunable values, constants, and hardware names in one place.
 */
@Configurable
object RobotConfig {

    // ==================== ALLIANCE & FIELD ====================
    @JvmField var alliance = Alliance.RED

    // Field dimensions (inches)
    const val FIELD_SIZE = 144.0

    // Goal positions (will be adjusted based on alliance)
    const val GOAL_Y = FIELD_SIZE  // 136.0 inches
    const val RED_GOAL_X = FIELD_SIZE  // 138.0 inches
    const val BLUE_GOAL_X = 0.0

    // Robot dimensions
    const val ROBOT_WIDTH = 14.358268
    const val ROBOT_LENGTH = 12.9921

    // ==================== HARDWARE NAMES ====================
    object Hardware {
        // Drive motors (configured in Pedro)
        const val IMU = "imu"

        // Shooter hardware
        const val FLYWHEEL_1 = "fly1"
        const val FLYWHEEL_2 = "fly2"
        const val TURRET_MOTOR = "turret"
        const val TURRET_LIMIT_SWITCH = "turretLimit"  // Touch sensor for homing
        const val HOOD_SERVO = "hood"

        // Intake hardware
        const val INTAKE = "intake"

        const val GATE_SERVO = "gate"

        // Vision
        const val LIMELIGHT = "ll"
    }

    // ==================== FLYWHEEL CONFIG ====================
    @Configurable
    object FlywheelConfig {

        @JvmField var pid = PIDCoefficients(
            0.08,    // moderate P — strong enough to respond, low enough to avoid wild oscillation
            0.0,     // keep 0 (integrator usually hurts flywheels — windup/overshoot)
            0.005   // tiny D to dampen any bounce (increase to 0.01–0.03 only if oscillating)
        )

        @JvmField var feedforward = BasicFeedforwardParameters(
            1.0 / 1950.0,   // tune this! Measure your motor's real free RPM at 100% power (no load)
            // Typical FTC shooter motors (550/Neo/etc geared) → 1800–2200 RPM free
            // Example: if measured 1980 RPM free → use 1.0 / 1980.0
            0.07,           // static friction — start 0.05–0.10; increase until it spins up from 0 RPM reliably
            0.0             // accel usually 0 for flywheels (no big inertia changes)
        )
        const val MOTOR_TICKS_PER_REV = 28.0
        const val WHEEL_DIAMETER_IN = 3.0
        const val WHEEL_RADIUS_IN = WHEEL_DIAMETER_IN / 2.0

        @JvmField var velocityTolerance = 25.0  // ticks/sec tolerance for "at speed"
    }

    // ==================== TURRET CONFIG ====================
    @Configurable
    object TurretConfig {
        @JvmField var pid = PIDCoefficients(0.3, 0.0, 0.15)

        const val GEAR_RATIO = 3.62068965517  // 105/29
        const val MOTOR_TICKS_PER_REV = 537.7
        val RADIANS_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)

        // Turret limits (radians)
        // Total range of 270 degrees (135 left, 135 right)
        const val MIN_ANGLE = -3 * PI / 4  // -2.356 radians
        const val MAX_ANGLE = 3 * PI / 4   //  2.356 radians
        @JvmField var manualPowerSlow = 0.5
        @JvmField var manualPowerFast = 1.0
    }

    // ==================== HOOD CONFIG ====================
    @Configurable
    object HoodConfig {
        const val MIN_POSITION = 0.0
        const val MAX_POSITION = 1.0
        @JvmField var adjustStep = 0.05
    }

    // ==================== LIMELIGHT CONFIG ====================
    @Configurable
    object LimelightConfig {
        @JvmField var mountAngleDeg = 10.0           // Tilt angle from horizontal
        @JvmField var lensHeightIn = 13.2101838583          // Height of lens from ground
        @JvmField var goalHeightIn = 30.0           // Height of scoring target

        const val POLL_RATE_HZ = 100
        const val DEFAULT_PIPELINE = 0
    }

    // ==================== PHYSICS CONFIG ====================
    @Configurable
    object Physics {
        const val GRAVITY_IN_PER_S2 = 386.0
        @JvmField var launchAngleDeg = 34.36
        @JvmField var shooterHeightIn = 12.446
    }

    // ==================== INTAKE CONFIG ====================
    @Configurable
    object IntakeConfig {
        @JvmField var intakePower = 1.0
        @JvmField var ejectPower = -1.0
        @JvmField var ejectSlowPower = -0.5
        @JvmField var feedPower = 0.8
    }

    // ==================== GATE CONFIG ====================
    @Configurable
    object GateConfig {
        @JvmField var openPosition = 0.0
        @JvmField var closedPosition = 1.0
    }

    // ==================== TIMING CONFIG ====================
    object Timing {
        const val FLYWHEEL_SPINUP_TIMEOUT_MS = 2000L
        const val FEED_DURATION_MS = 500L
        const val SHOOT_SEQUENCE_DELAY_MS = 400L
    }
}