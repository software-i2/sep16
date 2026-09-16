// Copyright by BeeX [2026]

#ifndef TASK_STATEMACHINE_H
#define TASK_STATEMACHINE_H

namespace task {

enum class State { IDLE, COLLECT, PROCESS, PLAN, EXECUTE, JAWCLOSING, DONE, RETRY, FAILED, STOPPED };

enum class Event {
    NONE,              // nothing happened this tick
    START,             // task/start
    STOP,              // task/stop
    PROCESSING,        // the cloud node has its frames and is processing them
    CANDIDATES_FOUND,
    NO_CANDIDATES,
    PLAN_FOUND,
    NO_PLAN,
    REACHED,           // the executor reached the grasp
    JAW_SETTLED,       // the jaw closed and stopped moving, or ran out of time
    RETRY_DUE,
    GAVE_UP,           // max_attempts used up
    FAILURE            // a node refused or failed
};

const char *stateName(State state);

// The state `event` leads to from `state`; `state` itself when the event does not apply there.
State nextState(State state, Event event);

}  // namespace task

#endif  // TASK_STATEMACHINE_H
