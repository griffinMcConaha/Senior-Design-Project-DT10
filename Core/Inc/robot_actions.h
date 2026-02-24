// Robot control task declarations (called by state machine)

#ifndef ROBOT_ACTIONS_H
#define ROBOT_ACTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

// Operator-controlled movement task (STATE_MANUAL)
void ManualControl_Task(void);

// GPS-guided autonomous mission task (STATE_AUTO)
void AutonomousControl_Task(void);

// Stop all motors immediately (fail-safe)
void Stop_Motors(void);

// Handle system error state (STATE_ERROR)
void Handle_Error(void);

// Emergency stop handler (STATE_ESTOP)
void Emergency_Stop(void);

// Parse RC speed command "M1:<speed> M2:<speed>" and set motor speeds
void ParseManualSpeedCommand(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_ACTIONS_H
