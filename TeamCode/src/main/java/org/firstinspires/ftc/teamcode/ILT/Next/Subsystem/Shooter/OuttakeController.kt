package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Aimbot.AimbotTable
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import org.firstinspires.ftc.teamcode.robot.data.enums.OuttakeMode
import org.firstinspires.ftc.teamcode.robot.data.enums.TurretMode

/**
 * OuttakeController - Coordinates all shooter subsystems.
 *
 * ============================================================
 * HOW THIS WORKS WITH LIMELIGHT + PEDRO (NO TURRET ENCODER)
 * ============================================================
 *
 * TURRET AIMING (left/right):
 * - Uses LIMELIGHT TX only
 * - TX tells us exactly how far off we are from target
 * - No encoder needed - Limelight IS our feedback!
 *
 * HOOD + FLYWHEEL (shot angle/power):
 * - Uses DISTANCE to determine settings
 * - Distance comes from: Limelight TY (preferred) OR Pedro odometry (fallback)
 * - Look up hood angle and flywheel speed in AimbotTable
 *
 * PEDRO ODOMETRY role:
 * - Tells us robot position (for distance calculation if LL fails)
 * - Tells us if we're in a valid shooting zone
 * - Does NOT directly control turret (we have no encoder to aim blindly)
 */
object OuttakeController : Subsystem {

    // ==================== MANUAL AIM SETTINGS ====================
    var manualAimDistance = 48  // inches (for MANUAL mode)
        private set

    // Track mode changes
    private var prevOuttakeMode: OuttakeMode? = null
    private var prevTurretMode: TurretMode? = null

    // ==================== PERIODIC ====================
    override fun periodic() {
        val outtakeModeChanged = RobotState.outtakeMode != prevOuttakeMode
        val turretModeChanged = RobotState.turretMode != prevTurretMode

        // ========================================
        // HANDLE OUTTAKE MODES (Hood + Flywheel)
        // ========================================
        when (RobotState.outtakeMode) {

            OuttakeMode.IDLE -> {
                if (outtakeModeChanged) {
                    RobotState.hoodPosition = 0.0
                    RobotState.flywheelOn = false
                }
            }

            OuttakeMode.MANUAL -> {
                applyManualAim()
            }

            OuttakeMode.AUTO_ODOMETRY -> {
                // Use Pedro distance for hood/flywheel
                if (RobotState.poseValid) {
                    applyAimFromDistance(RobotState.distanceToGoalOdometry, "Odometry")
                } else {
                    ActiveOpMode.telemetry.addData("Auto Aim", "No pose!")
                }
            }

            OuttakeMode.AUTO_LIMELIGHT -> {
                // Use Limelight distance (with odometry fallback)
                applyLimelightAim()
            }
        }

        // ========================================
        // HANDLE TURRET MODES
        // ========================================
        when (RobotState.turretMode) {
            TurretMode.IDLE -> Turret.currentState = Turret.State.IDLE

            TurretMode.MANUAL -> {
                Turret.currentState = Turret.State.MANUAL
                // manualPower is updated by the Gamepad/TeleOp directly
            }

            TurretMode.LIMELIGHT -> {
                // Just set the state; Turret.periodic() handles the "if hasTarget" check
                Turret.currentState = Turret.State.LIMELIGHT
            }

            TurretMode.ODOMETRY -> {
                // Strategy: Use Odom to find the general direction,
                // but if LL sees the target, let LL take over the state.
                if (RobotState.limelightHasTarget) {
                    Turret.currentState = Turret.State.LIMELIGHT
                } else {
                    Turret.currentState = Turret.State.ODOMETRY
                }
            }
        }

        //  Save for next loop
        prevOuttakeMode = RobotState.outtakeMode
        prevTurretMode = RobotState.turretMode

        // ========================================
        // TELEMETRY
        // ========================================
        ActiveOpMode.telemetry.run {
            addData("=== OUTTAKE CONTROLLER ===", "")
            addData("Outtake Mode", RobotState.outtakeMode)
            addData("Turret Mode", RobotState.turretMode)
            addData("Distance (Odom)", "%.1f\"".format(RobotState.distanceToGoalOdometry))
            addData("Distance (LL)", RobotState.distanceToGoalLimelight?.let { "%.1f\"".format(it) } ?: "N/A")
            addData("Ready to Shoot", canShoot())
        }
    }

    // ==================== AIM LOGIC ====================

    /**
     * Apply aim for MANUAL mode using manualAimDistance.
     */
    private fun applyManualAim() {
        val snapped = AimbotTable.snapToValidDistance(manualAimDistance)
        val values = AimbotTable.getValues(snapped.toDouble())

        if (values != null) {
            RobotState.hoodPosition = values.first
            FlyWheel.setTargetVelocity(values.second)
        }
    }

    /**
     * Apply aim from a distance value.
     */
    private fun applyAimFromDistance(distance: Double, source: String) {
        val values = AimbotTable.getValues(distance)

        if (values != null) {
            // Apply with offsets (tune these!)
            RobotState.hoodPosition = (values.first + 0.06).coerceIn(0.0, 1.0)
            FlyWheel.setTargetVelocity(values.second + 100)
            ActiveOpMode.telemetry.addData("Aim Source", source)
            ActiveOpMode.telemetry.addData("Aim Distance", "%.1f\"".format(distance))
        } else {
            ActiveOpMode.telemetry.addData("Aim", "Distance out of range: %.1f".format(distance))
        }
    }

    /**
     * Apply aim using Limelight distance, with odometry fallback.
     *
     * Priority:
     * 1. Limelight distance (most accurate when we can see target)
     * 2. Odometry distance (works when LL can't see target)
     */
    private fun applyLimelightAim() {
        val llDistance = RobotState.distanceToGoalLimelight

        when {
            // Limelight has distance - use it (best accuracy)
            llDistance != null && RobotState.limelightHasTarget -> {
                applyAimFromDistance(llDistance, "Limelight")
            }

            // Fallback to odometry
            RobotState.poseValid -> {
                applyAimFromDistance(RobotState.distanceToGoalOdometry, "Odometry (fallback)")
            }

            // Nothing works
            else -> {
                ActiveOpMode.telemetry.addData("Aim", "No distance source!")
            }
        }
    }

    // ==================== HELPER FUNCTIONS ====================

    /**
     * Check if we're ready to shoot.
     *
     * Requirements:
     * - Flywheel at speed
     * - Turret aligned (Limelight TX ≈ 0)
     */
    fun canShoot(): Boolean {
        return RobotState.flywheelAtSpeed && RobotState.turretAligned
    }

    /**
     * Increase manual aim distance.
     */
    fun increaseManualDistance() {
        manualAimDistance = (manualAimDistance + 12).coerceAtMost(144)
    }

    /**
     * Decrease manual aim distance.
     */
    fun decreaseManualDistance() {
        manualAimDistance = (manualAimDistance - 12).coerceAtLeast(12)
    }

    // ==================== MODE COMMANDS ====================

    /**
     * IDLE - Stop everything.
     */
    val idleMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.IDLE
        RobotState.turretMode = TurretMode.IDLE
    }

    /**
     * MANUAL - Driver controls everything.
     */
    val manualMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.MANUAL
        RobotState.turretMode = TurretMode.MANUAL
    }

    /**
     * AUTO with ODOMETRY for both distance AND turret aiming.
     *
     * Turret uses encoder position (zeroed on init) + Limelight for fine-tuning.
     * Hood/Flywheel use odometry distance.
     */
    val autoOdometryMode = InstantCommand {
        if (!RobotState.poseValid) {
            ActiveOpMode.telemetry.addData("Mode", "No pose available!")
            return@InstantCommand
        }
        RobotState.outtakeMode = OuttakeMode.AUTO_ODOMETRY
        RobotState.turretMode = TurretMode.ODOMETRY
    }

    /**
     * AUTO with LIMELIGHT - BEST MODE FOR COMPETITION.
     *
     * - Turret: Tracks target using Limelight TX
     * - Hood/Flywheel: Uses Limelight distance (with odometry fallback)
     */
    val autoLimelightMode = InstantCommand {
        RobotState.outtakeMode = OuttakeMode.AUTO_LIMELIGHT
        RobotState.turretMode = TurretMode.LIMELIGHT
    }

    // Manual distance adjustment
    val aimUp = InstantCommand { increaseManualDistance() }
    val aimDown = InstantCommand { decreaseManualDistance() }
}


/*
 * ============================================================
 * QUICK REFERENCE: WHAT EACH MODE DOES
 * ============================================================
 *
 * IDLE:
 *   Turret: Stopped
 *   Hood: Reset to 0
 *   Flywheel: Off
 *
 * MANUAL:
 *   Turret: Joystick control
 *   Hood: Based on manualAimDistance
 *   Flywheel: Based on manualAimDistance
 *
 * AUTO_ODOMETRY:
 *   Turret: Limelight TX (can't aim blind without encoder!)
 *   Hood: Based on Pedro distance
 *   Flywheel: Based on Pedro distance
 *
 * AUTO_LIMELIGHT (RECOMMENDED):
 *   Turret: Limelight TX
 *   Hood: Limelight distance (or Pedro if LL fails)
 *   Flywheel: Limelight distance (or Pedro if LL fails)
 *
 *
 * ============================================================
 * TYPICAL MATCH FLOW
 * ============================================================
 *
 * 1. Start of match:
 *    - Turret physically pointing forward
 *    - Mode: IDLE
 *
 * 2. Driving to shooting position:
 *    - Pedro tracks robot position
 *    - OuttakeController in IDLE or preparing
 *
 * 3. Ready to shoot:
 *    - Press button → autoLimelightMode
 *    - Turret starts tracking (LL TX → 0)
 *    - Hood/flywheel adjust for distance
 *    - Wait for flywheelAtSpeed AND turretAligned
 *
 * 4. Shoot!
 *    - Feed ball to shooter
 *    - Repeat for more balls
 *
 * 5. Done shooting:
 *    - Press button → idleMode
 *    - Drive to next position
 */