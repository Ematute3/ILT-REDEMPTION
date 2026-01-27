package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums

/**
 * Controls how hood angle and flywheel speed are determined
 */
enum class OuttakeMode {
    IDLE,              // No automatic adjustment
    MANUAL,            // Operator controls hood/flywheel manually
    AUTO_ODOMETRY,     // Auto adjust using Pedro odometry distance
    AUTO_LIMELIGHT     // Auto adjust using Limelight distance
}

/**
 * Controls how the turret aims.
 *
 * LIMELIGHT: Uses camera TX to track target (works without homing)
 * ODOMETRY: Uses robot position to calculate aim (REQUIRES HOMING FIRST!)
 */
enum class TurretMode {
    IDLE,              // Turret stationary
    MANUAL,            // Direct joystick control
    LIMELIGHT,         // Track target using Limelight TX
    ODOMETRY           // Aim using robot position (requires homing!)
}

/**
 * Controls shooting behavior
 */
enum class ShootMode {
    IDLE,
    MANUAL,            // Driver triggers shots
    AUTO               // Automatic shooting when conditions met
}

/**
 * Intake state tracking
 */
enum class IntakeState {
    STOPPED,
    INTAKING,
    EJECTING,
    FEEDING
}

/**
 * Ball pattern detected by Limelight fiducials
 */
enum class Motif {
    GPP,   // Green Purple Purple
    PGP,   // Purple Green Purple
    PPG,   // Purple Purple Green
    NONE
}

/**
 * Ball color for sensor detection
 */
enum class BallColor {
    GREEN,
    PURPLE
}