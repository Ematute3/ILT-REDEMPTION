package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.*

@Configurable
object Turret : Subsystem {

    enum class State { IDLE, MANUAL, LIMELIGHT, ODOMETRY, FUSED }

    val motor = MotorEx(RobotConfig.Hardware.TURRET_MOTOR)

    var controller = controlSystem {
        posPid(RobotConfig.TurretConfig.pid)
        basicFF(RobotConfig.TurretConfig.feedForward)
    }

    @JvmField var manualPower = 0.0
    @JvmField var currentState = State.IDLE

    // ==================== TUNING PARAMETERS ====================
    @JvmField var minPower: Double = 0.10  // Reduced - let controller handle most of it
    @JvmField var maxPower: Double = 1.0 // Increased ceiling for faster tracking
    @JvmField var alignmentTolerance: Double = 2.5  // Increased - was too tight
    @JvmField var visionGain: Double = 0.4
    @JvmField var threshold: Double = 0.1

    // NEW: Robot rotation compensation
    @JvmField var useRobotVelocityCompensation: Boolean = true
    @JvmField var robotVelocityGain: Double = 1.0  // Tune this if tracking is still off

    // Velocity tracking
    private var lastYaw: Double = 0.0
    private var lastTime: Long = System.nanoTime()
    private var currentVelocity: Double = 0.0

    // Robot heading tracking for velocity estimation
    private var lastRobotHeading: Double = 0.0
    private var lastHeadingTime: Long = System.nanoTime()
    private var robotAngularVelocity: Double = 0.0

    // Motion profiling
    @JvmField var maxVelocity: Double = 4.0  // Increased from 3.0
    @JvmField var maxAcceleration: Double = 8.0  // Increased from 6.0
    @JvmField var useMotionProfile: Boolean = true  // Changed to true

    private const val RADIANS_PER_TICK = 2.0 * PI /
            (RobotConfig.TurretConfig.MOTOR_TICKS_PER_REV * RobotConfig.TurretConfig.GEAR_RATIO)

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        lastYaw = getYaw()
        lastTime = System.nanoTime()
        lastRobotHeading = RobotState.currentHeading
        lastHeadingTime = System.nanoTime()
    }

    override fun periodic() {
        updateVelocity()
        updateRobotVelocity()  // NEW

        RobotState.turretYaw = getYaw()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.LIMELIGHT -> aimWithLimelightOnly()
            State.ODOMETRY -> aimWithOdometryOnly()
            State.FUSED -> fusedAimingLogic()
        }
    }

    /**
     * NEW: Track robot rotation velocity
     * This is critical for compensating when the robot spins
     */
    private fun updateRobotVelocity() {
        val currentHeading = RobotState.currentHeading
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastHeadingTime) / 1e9

        if (dt > 0) {
            var deltaHeading = currentHeading - lastRobotHeading

            // Normalize to [-PI, PI]
            if (deltaHeading > PI) deltaHeading -= 2.0 * PI
            if (deltaHeading < -PI) deltaHeading += 2.0 * PI

            robotAngularVelocity = deltaHeading / dt
        }

        lastRobotHeading = currentHeading
        lastHeadingTime = currentTime
    }

    private fun updateVelocity() {
        val currentYaw = getYaw()
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastTime) / 1e9

        if (dt > 0) {
            var deltaYaw = currentYaw - lastYaw

            if (deltaYaw > PI) deltaYaw -= 2.0 * PI
            if (deltaYaw < -PI) deltaYaw += 2.0 * PI

            currentVelocity = deltaYaw / dt
        }

        lastYaw = currentYaw
        lastTime = currentTime
    }

    private fun fusedAimingLogic() {
        if (!RobotState.poseValid) {
            motor.power = 0.0
            return
        }

        // 1. Calculate base odometry target
        val deltaX = RobotState.goalX - RobotState.currentX
        val deltaY = RobotState.goalY - RobotState.currentY
        val fieldAngle = atan2(deltaY, deltaX)

        val robotHeading = normalizeAngle(RobotState.currentHeading)
        var targetYaw = normalizeAngle(fieldAngle - robotHeading)

        // 2. Apply Limelight correction
        if (RobotState.limelightHasTarget) {
            val tx = RobotState.limelightTx

            if (abs(tx) > threshold) {
                val visionCorrection = Math.toRadians(tx) * visionGain
                targetYaw = normalizeAngle(targetYaw + visionCorrection)
            }
        }

        applyControlWithVelocity(targetYaw)
    }

    private fun aimWithOdometryOnly() {
        if (!RobotState.poseValid) return

        val deltaX = RobotState.goalX - RobotState.currentX
        val deltaY = RobotState.goalY - RobotState.currentY
        val fieldAngle = atan2(deltaY, deltaX)
        val robotHeading = if (abs(RobotState.currentHeading) > 2.0 * PI)
            Math.toRadians(RobotState.currentHeading) else RobotState.currentHeading

        applyControlWithVelocity(normalizeAngle(fieldAngle - robotHeading))
    }

    private fun aimWithLimelightOnly() {
        if (!RobotState.limelightHasTarget) {
            motor.power = 0.0
            return
        }

        val tx = RobotState.limelightTx
        val currentYaw = getYaw()
        val targetYaw = normalizeAngle(currentYaw + Math.toRadians(tx))

        applyControlWithVelocity(targetYaw)
    }

    /**
     * IMPROVED: Now compensates for robot rotation
     */
    private fun applyControlWithVelocity(targetYaw: Double) {
        val clampedTarget = targetYaw.coerceIn(-3 * PI / 4, 3 * PI / 4)
        val currentYaw = getYaw()

        // Calculate desired velocity
        var desiredVelocity = if (useMotionProfile) {
            calculateProfiledVelocity(currentYaw, clampedTarget)
        } else {
            0.0
        }

        // KEY FIX: Add robot rotation compensation
        // When robot spins right, turret must spin left to stay on target
        if (useRobotVelocityCompensation) {
            desiredVelocity -= robotAngularVelocity * robotVelocityGain
        }

        // Set goal with position AND velocity
        controller.goal = KineticState(clampedTarget, desiredVelocity)

        // Calculate control output
        var power = controller.calculate(KineticState(currentYaw, currentVelocity))

        // IMPROVED friction compensation - only apply when error is significant
        val error = abs(clampedTarget - currentYaw)
        val errorDeg = Math.toDegrees(error)

        if (errorDeg > alignmentTolerance && abs(power) < minPower) {
            // Only add friction kick if controller output is too weak
            power = (if (power >= 0) 1.0 else -1.0) * minPower
        }

        motor.power = power.coerceIn(-maxPower, maxPower)

        // Check alignment with relaxed tolerance
        val visionCentered = !RobotState.limelightHasTarget || abs(RobotState.limelightTx) < 2.0
        RobotState.turretAligned = errorDeg < alignmentTolerance && visionCentered
    }

    private fun calculateProfiledVelocity(currentPos: Double, targetPos: Double): Double {
        var error = targetPos - currentPos

        if (error > PI) error -= 2.0 * PI
        if (error < -PI) error += 2.0 * PI

        val errorAbs = abs(error)
        val direction = if (error > 0) 1.0 else -1.0

        // Deceleration distance based on current velocity
        val decelDistance = (currentVelocity * currentVelocity) / (2.0 * maxAcceleration)

        val targetVelocity = if (errorAbs < decelDistance) {
            // Deceleration phase
            sqrt(2.0 * maxAcceleration * errorAbs) * direction
        } else {
            // Acceleration/constant velocity phase
            maxVelocity * direction
        }

        return targetVelocity.coerceIn(-maxVelocity, maxVelocity)
    }

    // ==================== UTILITIES ====================

    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)
    fun getYawDegrees(): Double = Math.toDegrees(getYaw())
    fun getVelocity(): Double = currentVelocity
    fun getVelocityDegPerSec(): Double = Math.toDegrees(currentVelocity)
    fun getRobotAngularVelocity(): Double = robotAngularVelocity  // NEW - for debugging

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    // ==================== COMMANDS ====================

    fun aimWithLimelight() { currentState = State.LIMELIGHT }
    fun aimWithOdometry() { currentState = State.ODOMETRY }
    fun aimWithBoth() { currentState = State.FUSED }
    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
    }
}