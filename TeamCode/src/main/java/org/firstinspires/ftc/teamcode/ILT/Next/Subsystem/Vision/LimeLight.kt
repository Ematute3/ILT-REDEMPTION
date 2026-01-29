package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Motif

import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Limelight vision subsystem.
 * Handles target detection, distance calculation, and fiducial tracking.
 */
object Limelight: Subsystem{

    private var limelight: Limelight3A? = null
    private var isInitialized = false

    // Fiducial data
    var fiducialCount: Int = 0
        private set
    var fiducialData: String = "No fiducials"
        private set

    override fun initialize() {
        try {
            limelight = ActiveOpMode.hardwareMap.get(
                Limelight3A::class.java,
                RobotConfig.Hardware.LIMELIGHT
            )

            limelight?.let { ll ->
                ll.setPollRateHz(RobotConfig.LimelightConfig.POLL_RATE_HZ)
                ll.pipelineSwitch(RobotConfig.LimelightConfig.DEFAULT_PIPELINE)
                ll.start()
                isInitialized = true
                RobotState.limelightReady = true
                ActiveOpMode.telemetry.addData("Limelight", "Initialized")
            }
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Limelight Error", e.message)
            isInitialized = false
            RobotState.limelightReady = false
        }
    }

    override fun periodic() {
        if (!isInitialized || limelight == null) {
            RobotState.limelightReady = false
            RobotState.limelightHasTarget = false
            return
        }

        RobotState.limelightReady = true

        val result = limelight?.latestResult

        if (result != null && result.isValid) {
            // Update basic targeting data
            RobotState.limelightHasTarget = true
            RobotState.limelightTx = result.tx
            RobotState.limelightTy = result.ty
            RobotState.limelightTa = result.ta

            // Calculate distance
            calculateDistance(result.ty)

            // Update fiducial data
            updateFiducialData(result)

            // Update motif detection
            updateMotifDetection(result)
        } else {
            RobotState.limelightHasTarget = false
            RobotState.limelightTx = 0.0
            RobotState.limelightTy = 0.0
            RobotState.limelightTa = 0.0
            RobotState.distanceToGoalLimelight = null
            RobotState.detectedMotif = Motif.NONE
            fiducialCount = 0
            fiducialData = "No valid result"
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("LL Ready", RobotState.limelightReady)
            addData("LL Target", RobotState.limelightHasTarget)
            if (RobotState.limelightHasTarget) {
                addData("LL TX", "%.2f°".format(RobotState.limelightTx))
                addData("LL TY", "%.2f°".format(RobotState.limelightTy))
                addData("LL Distance", RobotState.distanceToGoalLimelight?.let { "%.1f\"".format(it) } ?: "N/A")
            }
        }
    }

    /**
     * Calculate distance to target using vertical angle.
     */
    private fun calculateDistance(ty: Double) {
        val angleToGoalDeg = RobotConfig.LimelightConfig.mountAngleDeg + ty
        val angleToGoalRad = Math.toRadians(angleToGoalDeg)

        // Safety checks
        if (abs(angleToGoalRad) >= PI / 2) {
            RobotState.distanceToGoalLimelight = null
            return
        }

        val tanAngle = tan(angleToGoalRad)
        if (abs(tanAngle) < 0.001) {
            RobotState.distanceToGoalLimelight = null
            return
        }

        val heightDiff = RobotConfig.LimelightConfig.goalHeightIn - RobotConfig.LimelightConfig.lensHeightIn
        val distance = heightDiff / tanAngle

        // Sanity check
        if (distance > 0 && distance < 200.0) {
            RobotState.distanceToGoalLimelight = distance
        } else {
            RobotState.distanceToGoalLimelight = null
        }
    }

    /**
     * Update fiducial (AprilTag) data.
     */
    private fun updateFiducialData(result: LLResult) {
        val fiducials = result.fiducialResults
        fiducialCount = fiducials.size

        if (fiducials.isNotEmpty()) {
            fiducialData = fiducials.joinToString("\n") { fr ->
                "ID: ${fr.fiducialId}, X: ${"%.2f".format(fr.targetXDegrees)}°"
            }
        } else {
            fiducialData = "No fiducials detected"
        }
    }

    /**
     * Update motif detection from fiducial IDs.
     */
    private fun updateMotifDetection(result: LLResult) {
        val fiducials = result.fiducialResults
        if (fiducials.isNotEmpty()) {
            RobotState.detectedMotif = when (fiducials[0].fiducialId) {
                21 -> Motif.GPP
                22 -> Motif.PGP
                else -> Motif.PPG
            }
        } else {
            RobotState.detectedMotif = Motif.NONE
        }
    }

    /**
     * Calculate required flywheel RPM for distance using projectile physics.
     */
    fun calculateRequiredRPM(distance: Double): Double {
        val theta = Math.toRadians(RobotConfig.Physics.launchAngleDeg)
        val cosTheta = cos(theta)
        val tanTheta = tan(theta)
        val heightDiff = RobotConfig.LimelightConfig.goalHeightIn - RobotConfig.Physics.shooterHeightIn

        val numerator = RobotConfig.Physics.GRAVITY_IN_PER_S2 * distance * distance
        val denominator = 2.0 * cosTheta * cosTheta * (distance * tanTheta - heightDiff)

        if (denominator <= 0) return 0.0

        val velocityInPerSec = sqrt(numerator / denominator)
        return 60.0 * velocityInPerSec / (2.0 * PI * RobotConfig.FlywheelConfig.WHEEL_RADIUS_IN)
    }

    /**
     * Calculate required flywheel velocity in ticks/sec for distance.
     */
    fun calculateRequiredVelocity(distance: Double): Double {
        val rpm = calculateRequiredRPM(distance)
        if (rpm == 0.0) return 0.0
        return (rpm / 60.0) * RobotConfig.FlywheelConfig.MOTOR_TICKS_PER_REV
    }

    /**
     * Get latest result (for advanced use).
     */
    fun getLatestResult(): LLResult? {
        return if (isInitialized) limelight?.latestResult else null
    }

    /**
     * Switch pipeline.
     */
    fun setPipeline(pipeline: Int) {
        limelight?.pipelineSwitch(pipeline)
    }

    /**
     * Update robot orientation for MegaTag (pose estimation).
     */
    fun updateRobotOrientation(yawRadians: Double) {
        limelight?.updateRobotOrientation(Math.toDegrees(yawRadians))
    }
}