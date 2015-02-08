

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#define SM_STATE(machine, state) \
static void sm_ ## machine ## _ ## state ## _Enter(STATE_MACHINE_DATA *sm, \
	int global)

#define SM_ENTRY(machine, state) \
if (!global || sm->machine ## _state != machine ## _ ## state) { \
	sm->changed = TRUE; \
	wpa_printf(MSG_DEBUG, STATE_MACHINE_DEBUG_PREFIX ": " #machine \
		   " entering state " #state); \
} \
sm->machine ## _state = machine ## _ ## state;

#define SM_ENTRY_M(machine, _state, data) \
if (!global || sm->data ## _ ## state != machine ## _ ## _state) { \
	sm->changed = TRUE; \
	wpa_printf(MSG_DEBUG, STATE_MACHINE_DEBUG_PREFIX ": " \
		   #machine " entering state " #_state); \
} \
sm->data ## _ ## state = machine ## _ ## _state;

#define SM_ENTRY_MA(machine, _state, data) \
if (!global || sm->data ## _ ## state != machine ## _ ## _state) { \
	sm->changed = TRUE; \
	wpa_printf(MSG_DEBUG, STATE_MACHINE_DEBUG_PREFIX ": " MACSTR " " \
		   #machine " entering state " #_state, \
		   MAC2STR(STATE_MACHINE_ADDR)); \
} \
sm->data ## _ ## state = machine ## _ ## _state;

#define SM_ENTER(machine, state) \
sm_ ## machine ## _ ## state ## _Enter(sm, 0)

#define SM_ENTER_GLOBAL(machine, state) \
sm_ ## machine ## _ ## state ## _Enter(sm, 1)

#define SM_STEP(machine) \
static void sm_ ## machine ## _Step(STATE_MACHINE_DATA *sm)

#define SM_STEP_RUN(machine) sm_ ## machine ## _Step(sm)

#endif /* STATE_MACHINE_H */
