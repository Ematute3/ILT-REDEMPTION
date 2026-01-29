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
    @JvmField var visionGain: Double = 0.35
    @JvmField var visionDeadbandDeg: Double = 0.5   // Ignore |tx| < this to reduce jitter
    @JvmField var txFilterAlpha: Double = 0.25     // EMA: higher = less smoothing

    // Robot rotation compensation
    @JvmField var useRobotVelocityCompensation: Boolean = true
    @JvmField var robotVelocityGain: Double = 0.9
    private var filteredRobotAngularVelocity: Double = 0.0

    // Velocity tracking
    private var lastYaw: Double = 0.0
    private var lastTime: Long = System.nanoTime()
    private var currentVelocity: Double = 0.0

    private var lastRobotHeading: Double = 0.0
    private var lastHeadingTime: Long = System.nanoTime()

    // Motion profiling
    @JvmField var maxVelocity: Double = 3.5
    @JvmField var maxAcceleration: Double = 6.0
    @JvmField var useMotionProfile: Boolean = true
    @JvmField var nearTargetErrorDeg: Double = 5.0   // Cap desired vel when error < this
    @JvmField var nearTargetMaxVel: Double = 0.8    // rad/s when close

    // Filtered Limelight tx (EMA) to reduce oscillation from vision noise
    private var filteredTxDeg: Double = 0.0

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
        updateRobotVelocity()

        RobotState.turretYaw = getYaw()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.LIMELIGHT -> aimWithLimelightOnly()
            State.ODOMETRY -> aimWithOdometryOnly()
            State.FUSED -> fusedAimingLogic()
        }
    }

    /** Track robot rotation velocity; low-pass filtered to reduce noise. */
    private fun updateRobotVelocity() {
        val currentHeading = RobotState.currentHeading
        val currentTime = System.nanoTime()
        val dt = (currentTime - lastHeadingTime) / 1e9

        if (dt > 0) {
            var deltaHeading = currentHeading - lastRobotHeading
            if (deltaHeading > PI) deltaHeading -= 2.0 * PI
            if (deltaHeading < -PI) deltaHeading += 2.0 * PI
            val raw = deltaHeading / dt
            val alpha = 0.3
            filteredRobotAngularVelocity = alpha * raw + (1.0 - alpha) * filteredRobotAngularVelocity
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

        val deltaX = RobotState.goalX - RobotState.currentX
        val deltaY = RobotState.goalY - RobotState.currentY
        val fieldAngle = atan2(deltaY, deltaX)
        val robotHeading = normalizeAngle(RobotState.currentHeading)
        var targetYaw = normalizeAngle(fieldAngle - robotHeading)

        if (RobotState.limelightHasTarget) {
            updateFilteredTx(RobotState.limelightTx)
            if (abs(filteredTxDeg) > visionDeadbandDeg) {
                val visionCorrection = Math.toRadians(filteredTxDeg) * visionGain
                targetYaw = normalizeAngle(targetYaw + visionCorrection)
            }
        }

        applyControlWithVelocity(targetYaw)
    }

    private fun aimWithOdometryOnly() {
        if (!RobotState.poseValid) {
            motor.power = 0.0
            return
        }

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

        updateFilteredTx(RobotState.limelightTx)
        if (abs(filteredTxDeg) <= visionDeadbandDeg) {
            // Effectively aligned: hold current position to avoid jitter
            applyControlWithVelocity(getYaw())
            return
        }

        val currentYaw = getYaw()
        val targetYaw = normalizeAngle(currentYaw + Math.toRadians(filteredTxDeg))
        applyControlWithVelocity(targetYaw)
    }

    private fun updateFilteredTx(rawTxDeg: Double) {
        filteredTxDeg = txFilterAlpha * rawTxDeg + (1.0 - txFilterAlpha) * filteredTxDeg
    }

    /**
     * Shortest angular error in [-PI, PI]. Use this for tolerance checks and control.
     */
    private fun shortestAngularError(current: Double, target: Double): Double {
        var e = target - current
        while (e > PI) e -= 2.0 * PI
        while (e < -PI) e += 2.0 * PI
        return e
    }

    private fun applyControlWithVelocity(targetYaw: Double) {
        // 270° total range: ±135° from center (TurretConfig.MIN/MAX_ANGLE)
        val clampedTarget = targetYaw.coerceIn(
            RobotConfig.TurretConfig.MIN_ANGLE,
            RobotConfig.TurretConfig.MAX_ANGLE
        )
        val currentYaw = getYaw()
        val errorRad = shortestAngularError(currentYaw, clampedTarget)
        val errorDeg = Math.toDegrees(abs(errorRad))

        var desiredVelocity = if (useMotionProfile) {
            calculateProfiledVelocity(currentYaw, clampedTarget)
        } else {
            0.0
        }

        if (errorDeg < nearTargetErrorDeg) {
            val sign = if (errorRad >= 0) 1.0 else -1.0
            desiredVelocity = sign * minOf(abs(desiredVelocity), nearTargetMaxVel)
        }

        if (useRobotVelocityCompensation) {
            desiredVelocity -= filteredRobotAngularVelocity * robotVelocityGain
        }

        controller.goal = KineticState(clampedTarget, desiredVelocity)
        var power = controller.calculate(KineticState(currentYaw, currentVelocity))

        // Friction kick only when FAR from target; otherwise it causes limit-cycle oscillation
        val farFromTarget = errorDeg > 12.0
        if (farFromTarget && abs(power) < minPower) {
            power = (if (power >= 0) 1.0 else -1.0) * minPower
        }

        if (errorDeg < 1.0 && abs(power) < 0.03) {
            power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)

        val txForAlign = if (RobotState.limelightHasTarget) abs(filteredTxDeg) else 0.0
        val visionCentered = !RobotState.limelightHasTarget || txForAlign < 2.0
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
    fun getRobotAngularVelocity(): Double = filteredRobotAngularVelocity

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