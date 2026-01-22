package org.firstinspires.ftc.teamcode.robot.subsystems.drive

import com.pedropathing.control.PIDFController
import com.pedropathing.math.MathFunctions
import dev.nextftc.core.commands.Command
import dev.nextftc.extensions.pedro.PedroComponent
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.atan2

/**
 * Drive command with heading lock capability.
 * Robot can be driven normally while automatically maintaining heading toward a goal.
 */
class HeadingLock(
    private val drivePower: () -> Double,
    private val strafePower: () -> Double,
    private val turnPower: () -> Double,
    private val robotCentric: Boolean = false
) : Command() {

    private var controller: PIDFController? = null

    var headingLockEnabled: Boolean = false

    // Custom goal override (null = use RobotState goal)
    private var customGoalX: Double? = null
    private var customGoalY: Double? = null

    override val isDone: Boolean = false  // Runs continuously

    override fun start() {
        PedroComponent.follower?.let { follower ->
            controller = PIDFController(follower.constants.coefficientsHeadingPIDF)
            follower.startTeleopDrive()
        }
    }

    override fun update() {
        val follower = PedroComponent.follower ?: return

        val drive = drivePower()
        val strafe = strafePower()
        val turn = turnPower()

        // Update controller coefficients
        controller?.setCoefficients(follower.constants.coefficientsHeadingPIDF)

        if (headingLockEnabled && controller != null && RobotState.poseValid) {
            try {
                val error = calculateHeadingError()
                controller!!.updateError(error)
                follower.setTeleOpDrive(
                    drive,
                    strafe,
                    controller!!.run(),
                    robotCentric
                )
            } catch (e: Exception) {
                // Fallback to manual if calculation fails
                follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
            }
        } else {
            follower.setTeleOpDrive(drive, strafe, turn, robotCentric)
        }
    }

    override fun stop(interrupted: Boolean) {
        if (interrupted) {
            PedroComponent.follower?.breakFollowing()
        }
    }

    /**
     * Calculate heading error to face the goal.
     */
    private fun calculateHeadingError(): Double {
        val follower = PedroComponent.follower ?: return 0.0
        val currentPose = follower.pose

        // Use custom goal if set, otherwise use RobotState goal
        val targetX = customGoalX ?: RobotState.goalX
        val targetY = customGoalY ?: RobotState.goalY

        // Calculate desired heading to face goal
        val desiredHeading = atan2(targetY - currentPose.y, targetX - currentPose.x)

        // Use Pedro's math functions for consistent angle handling
        return MathFunctions.getTurnDirection(currentPose.heading, desiredHeading) *
                MathFunctions.getSmallestAngleDifference(currentPose.heading, desiredHeading)
    }

    /**
     * Set a custom goal position for heading lock.
     */
    fun setCustomGoal(x: Double, y: Double) {
        customGoalX = x
        customGoalY = y
    }

    /**
     * Clear custom goal (revert to RobotState goal).
     */
    fun clearCustomGoal() {
        customGoalX = null
        customGoalY = null
    }

    /**
     * Check if heading lock can function.
     */
    fun canHeadingLock(): Boolean {
        return PedroComponent.follower != null &&
                controller != null &&
                RobotState.poseValid
    }
}