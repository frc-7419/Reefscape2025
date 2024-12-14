package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.Elastic.ElasticNotification;
import frc.robot.util.Elastic.ElasticNotification.NotificationLevel;

/**
 * A wrapper that integrates WPILib's Alert with the Elastic dashboard
 * notification system.
 * It sends notifications when the alert transitions from off to on.
 */
public class CombinedAlert {
    private final Alert wpilibAlert;
    private final ElasticNotification notification;

    private boolean lastState = false; // Track previous set state to avoid spamming

    /**
     * The severity of the alert, determining the AlertType and NotificationLevel.
     */
    public enum Severity {
        INFO,
        WARNING,
        ERROR
    }

    // Main Constructor (All parameters) //

    /**
     * Creates a CombinedAlert with all parameters specified.
     *
     * @param severity      The severity level of this alert.
     * @param title         The title text of the notification.
     * @param description   The description text of the notification.
     * @param displayTimeMs The time in milliseconds the notification is displayed
     *                      (0 = no auto-dismiss).
     * @param width         The width of the notification.
     * @param height        The height of the notification (-1 for automatic
     *                      height).
     */
    public CombinedAlert(Severity severity, String title, String description, int displayTimeMs, double width,
            double height) {
        // Create WPILib Alert
        this.wpilibAlert = new Alert(title, mapSeverityToWpilibType(severity));

        // Create the ElasticNotification once and store it
        this.notification = new ElasticNotification(
                mapSeverityToElasticLevel(severity),
                title,
                description,
                displayTimeMs,
                width,
                height);
    }

    // Convenience Constructors (mirroring ElasticNotification) //

    /**
     * Creates a CombinedAlert with a specified severity, title, and description,
     * using default display time (3000 ms), default width (350), and automatic
     * height (-1).
     * 
     * @param severity    The severity level of this alert.
     * @param title       The title text of the notification.
     * @param description The description text of the notification.
     * 
     */
    public CombinedAlert(Severity severity, String title, String description) {
        this(severity, title, description, 3000, 350, -1);
    }

    /**
     * Creates a CombinedAlert with a specified severity, title, and description,
     * and a custom display time.
     * Uses default width (350) and automatic height (-1).
     *
     * @param severity      The severity level of this alert.
     * @param title         The title text of the notification.
     * @param description   The description text of the notification.
     * @param displayTimeMs The display time in milliseconds.
     */
    public CombinedAlert(Severity severity, String title, String description, int displayTimeMs) {
        this(severity, title, description, displayTimeMs, 350, -1);
    }

    /**
     * Creates a CombinedAlert with a specified severity, title, and description,
     * and custom width/height.
     * Uses default display time (3000 ms).
     *
     * @param severity    The severity level of this alert.
     * @param title       The title text of the notification.
     * @param description The description text of the notification.
     * @param width       The width of the notification.
     * @param height      The height of the notification (-1 for automatic height).
     */
    public CombinedAlert(Severity severity, String title, String description, double width, double height) {
        this(severity, title, description, 3000, width, height);
    }

    /**
     * Sets the state of the combined alert.
     *
     * @param active If true, displays the alert and sends a notification (if just
     *               transitioned to true).
     *               If false, hides the alert.
     */
    public void set(boolean active) {
        wpilibAlert.set(active);

        // If transitioning from false to true, send the Elastic notification
        if (active && !lastState) {
            sendElasticNotification();
        }

        lastState = active;
    }

    // Helper methods //
    private void sendElasticNotification() {
        Elastic.sendAlert(notification);
    }

    private static AlertType mapSeverityToWpilibType(Severity severity) {
        switch (severity) {
            case ERROR:
                return AlertType.kError;
            case WARNING:
                return AlertType.kWarning;
            case INFO:
            default:
                return AlertType.kInfo;
        }
    }

    private static NotificationLevel mapSeverityToElasticLevel(Severity severity) {
        switch (severity) {
            case ERROR:
                return NotificationLevel.ERROR;
            case WARNING:
                return NotificationLevel.WARNING;
            case INFO:
            default:
                return NotificationLevel.INFO;
        }
    }
}