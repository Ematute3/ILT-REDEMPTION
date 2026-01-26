package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.Aimbot
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Auto-Aim Calculator
 *
 * Calculates optimal hood position and flywheel velocity based on distance to target.
 * Uses both the regression-based Aimbot.java lookup and the discrete AimbotTable.kt
 * for maximum accuracy.
 *
 * Features:
 * - Automatic distance calculation from odometry
 * - Automatic distance calculation from Limelight
 * - Interpolation between known data points
 * - Configurable offsets for fine-tuning
 * - Safety bounds checking
 */
object AutoAimCalculator {

    /**
     * Data class to hold calculated aim parameters
     */
    data class AimParameters(
        val hoodPosition: Double,      // 0.0 to 1.0
        val flywheelVelocity: Double,  // ticks per second
        val distance: Double,          // inches
        val source: AimSource           // Which calculation method was used
    )

    enum class AimSource {
        ODOMETRY_REGRESSION,    // Using Aimbot.java with odometry distance
        ODOMETRY_TABLE,         // Using AimbotTable.kt with odometry distance
        LIMELIGHT_REGRESSION,   // Using Aimbot.java with Limelight distance
        LIMELIGHT_TABLE,        // Using AimbotTable.kt with Limelight distance
        MANUAL,                 // Manual distance input
        FALLBACK                // Safe defaults when no data available
    }

    // ==================== CONFIGURATION ====================

    /**
     * Tunable offsets - adjust these during testing
     */
    @JvmField var hoodOffset: Double = 0.06      // Added to calculated hood position
    @JvmField var velocityOffset: Double = 100.0  // Added to calculated velocity

    /**
     * Choose which lookup method to use
     */
    enum class LookupMethod {
        REGRESSION,  // Use Aimbot.java (smooth interpolation across all distances)
        TABLE,       // Use AimbotTable.kt (discrete points with linear interpolation)
        AUTO         // Automatically choose best method based on distance
    }

    @JvmField var preferredMethod: LookupMethod = LookupMethod.AUTO

    /**
     * Safety limits
     */
    private const val MIN_HOOD_POSITION = 0.0
    private const val MAX_HOOD_POSITION = 1.0
    private const val MIN_FLYWHEEL_VELOCITY = 0.0
    private const val MAX_FLYWHEEL_VELOCITY = 2500.0

    /**
     * Distance limits (based on your data)
     */
    private const val MIN_DISTANCE = 12.0
    private const val MAX_DISTANCE = 198.0

    // ==================== MAIN CALCULATION FUNCTIONS ====================

    /**
     * Calculate aim parameters using current odometry position
     */
    fun calculateFromOdometry(): AimParameters? {
        if (!RobotState.poseValid) return null

        val distance = calculateDistanceToGoal(
            RobotState.currentX,
            RobotState.currentY,
            RobotState.goalX,
            RobotState.goalY
        )

        return calculateFromDistance(distance, useOdometry = true)
    }

    /**
     * Calculate aim parameters using Limelight distance
     */
    fun calculateFromLimelight(): AimParameters? {
        val llDistance = RobotState.distanceToGoalLimelight ?: return null

        return calculateFromDistance(llDistance, useOdometry = false)
    }

    /**
     * Calculate aim parameters from a specific distance
     * This is the core calculation function
     */
    fun calculateFromDistance(distance: Double, useOdometry: Boolean = true): AimParameters {
        // Clamp distance to valid range
        val clampedDistance = distance.coerceIn(MIN_DISTANCE, MAX_DISTANCE)

        // Choose calculation method
        val (hood, velocity, source) = when (preferredMethod) {
            LookupMethod.REGRESSION -> {
                val values = calculateFromRegression(clampedDistance)
                Triple(
                    values[0],
                    values[1],
                    if (useOdometry) AimSource.ODOMETRY_REGRESSION else AimSource.LIMELIGHT_REGRESSION
                )
            }

            LookupMethod.TABLE -> {
                val (h, v) = calculateFromTable(clampedDistance)
                Triple(
                    h,
                    v,
                    if (useOdometry) AimSource.ODOMETRY_TABLE else AimSource.LIMELIGHT_TABLE
                )
            }

            LookupMethod.AUTO -> {
                // Use table for known points, regression for in-between
                if (isKnownTableDistance(clampedDistance)) {
                    val (h, v) = calculateFromTable(clampedDistance)
                    Triple(
                        h,
                        v,
                        if (useOdometry) AimSource.ODOMETRY_TABLE else AimSource.LIMELIGHT_TABLE
                    )
                } else {
                    val values = calculateFromRegression(clampedDistance)
                    Triple(
                        values[0],
                        values[1],
                        if (useOdometry) AimSource.ODOMETRY_REGRESSION else AimSource.LIMELIGHT_REGRESSION
                    )
                }
            }
        }

        // Apply offsets
        val finalHood = (hood + hoodOffset).coerceIn(MIN_HOOD_POSITION, MAX_HOOD_POSITION)
        val finalVelocity = (velocity + velocityOffset).coerceIn(MIN_FLYWHEEL_VELOCITY, MAX_FLYWHEEL_VELOCITY)

        return AimParameters(finalHood, finalVelocity, clampedDistance, source)
    }

    /**
     * Calculate best aim parameters using all available sensors
     * Prioritizes Limelight if available, falls back to odometry
     */
    fun calculateBestAvailable(): AimParameters {
        // Try Limelight first (most accurate)
        calculateFromLimelight()?.let { return it }

        // Fall back to odometry
        calculateFromOdometry()?.let { return it }

        // Last resort: use middle distance as fallback
        return createFallbackParameters()
    }

    // ==================== CALCULATION METHODS ====================

    /**
     * Use Aimbot.java regression model
     * Provides smooth interpolation across all distances
     */
    private fun calculateFromRegression(distance: Double): DoubleArray {
        return Aimbot.getValues(distance)
    }

    /**
     * Use AimbotTable.kt discrete lookup
     * Provides exact values at known distances with linear interpolation
     */
    private fun calculateFromTable(distance: Double): Pair<Double, Double> {
        return AimbotTable.getValues(distance)
    }

    /**
     * Check if distance is exactly a known table point (multiples of 12)
     */
    private fun isKnownTableDistance(distance: Double): Boolean {
        val validDistances = AimbotTable.getValidDistances()
        return validDistances.any { kotlin.math.abs(it - distance) < 0.1 }
    }

    /**
     * Calculate Euclidean distance to goal
     */
    private fun calculateDistanceToGoal(
        currentX: Double,
        currentY: Double,
        goalX: Double,
        goalY: Double
    ): Double {
        val deltaX = goalX - currentX
        val deltaY = goalY - currentY
        return sqrt(deltaX.pow(2) + deltaY.pow(2))
    }

    /**
     * Create safe fallback parameters when no sensors available
     */
    private fun createFallbackParameters(): AimParameters {
        // Use middle distance (72 inches) as reasonable fallback
        val fallbackDistance = 72.0
        val values = calculateFromRegression(fallbackDistance)

        return AimParameters(
            hoodPosition = (values[0] + hoodOffset).coerceIn(MIN_HOOD_POSITION, MAX_HOOD_POSITION),
            flywheelVelocity = (values[1] + velocityOffset).coerceIn(MIN_FLYWHEEL_VELOCITY, MAX_FLYWHEEL_VELOCITY),
            distance = fallbackDistance,
            source = AimSource.FALLBACK
        )
    }

    // ==================== CONVENIENCE FUNCTIONS ====================

    /**
     * Get just the hood position for current conditions
     */
    fun getHoodPosition(): Double {
        return calculateBestAvailable().hoodPosition
    }

    /**
     * Get just the flywheel velocity for current conditions
     */
    fun getFlywheelVelocity(): Double {
        return calculateBestAvailable().flywheelVelocity
    }

    /**
     * Get calculated distance to goal from odometry
     */
    fun getDistanceToGoal(): Double? {
        if (!RobotState.poseValid) return null

        return calculateDistanceToGoal(
            RobotState.currentX,
            RobotState.currentY,
            RobotState.goalX,
            RobotState.goalY
        )
    }

    /**
     * Apply calculated parameters to subsystems
     */
    fun applyToSubsystems() {
        val params = calculateBestAvailable()

        Hood.setPosition(params.hoodPosition)
        FlyWheel.setTargetVelocity(params.flywheelVelocity)

        // Update robot state for telemetry
        RobotState.distanceToGoalOdometry = params.distance
    }

    /**
     * Apply specific parameters to subsystems
     */
    fun applyParameters(params: AimParameters) {
        Hood.setPosition(params.hoodPosition)
        FlyWheel.setTargetVelocity(params.flywheelVelocity)
    }

    // ==================== VALIDATION & DEBUG ====================

    /**
     * Validate that a distance is within acceptable range
     */
    fun isDistanceValid(distance: Double): Boolean {
        return distance in MIN_DISTANCE..MAX_DISTANCE
    }

    /**
     * Get detailed calculation info for debugging
     */
    fun getDebugInfo(): String {
        val params = calculateBestAvailable()

        return buildString {
            appendLine("=== AUTO AIM CALCULATOR ===")
            appendLine("Source: ${params.source}")
            appendLine("Distance: ${"%.1f".format(params.distance)}\"")
            appendLine("Hood Position: ${"%.3f".format(params.hoodPosition)}")
            appendLine("Flywheel Velocity: ${"%.0f".format(params.flywheelVelocity)} ticks/sec")
            appendLine("Flywheel RPM: ${"%.0f".format(params.flywheelVelocity * 60.0 / 28.0)}")
            appendLine()
            appendLine("Offsets:")
            appendLine("  Hood: +${"%.3f".format(hoodOffset)}")
            appendLine("  Velocity: +${"%.0f".format(velocityOffset)}")
            appendLine()
            appendLine("Sensors:")
            appendLine("  Pose Valid: ${RobotState.poseValid}")
            appendLine("  LL Distance: ${RobotState.distanceToGoalLimelight?.let { "%.1f\"".format(it) } ?: "N/A"}")

            if (RobotState.poseValid) {
                appendLine("  Odom Distance: ${"%.1f\"".format(getDistanceToGoal())}")
            }
        }
    }

    /**
     * Compare regression vs table methods for a given distance
     */
    fun compareCalculationMethods(distance: Double): String {
        val regression = calculateFromRegression(distance)
        val table = calculateFromTable(distance)

        return buildString {
            appendLine("=== COMPARISON at ${"%.1f".format(distance)}\" ===")
            appendLine("Regression (Aimbot.java):")
            appendLine("  Hood: ${"%.3f".format(regression[0])}")
            appendLine("  Velocity: ${"%.0f".format(regression[1])}")
            appendLine()
            appendLine("Table (AimbotTable.kt):")
            appendLine("  Hood: ${"%.3f".format(table.first)}")
            appendLine("  Velocity: ${"%.0f".format(table.second)}")
            appendLine()
            appendLine("Difference:")
            appendLine("  Hood: ${"%.3f".format(kotlin.math.abs(regression[0] - table.first))}")
            appendLine("  Velocity: ${"%.0f".format(kotlin.math.abs(regression[1] - table.second))}")
        }
    }

    /**
     * Get all valid shooting distances from table
     */
    fun getValidDistances(): List<Int> {
        return AimbotTable.getValidDistances()
    }
}