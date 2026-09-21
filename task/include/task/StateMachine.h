// Copyright by BeeX [2026]

#ifndef TASK_STATEMACHINE_H
#define TASK_STATEMACHINE_H

namespace task {

// RECOLLECT, REPROCESS, REPLAN and REVERIFY mirror COLLECT, PROCESS, PLAN and RETRY, run once
// the vehicle has parked. The pick is planned from what the camera sees there, so nothing the
// drive got wrong is carried into the grasp.
enum class State {
    IDLE,
    COLLECT,
    PROCESS,
    PARK,
    PLAN,
    RECOLLECT,
    REPROCESS,
    REPLAN,
    REVERIFY,
    EXECUTE,
    JAWCLOSING,
    DONE,
    RETRY,
    FAILED,
    STOPPED
};

enum class Event {
    NONE,              // nothing happened this tick
    START,             // task/start
    STOP,              // task/stop
    PROCESSING,        // the cloud node has its frames and is processing them
    CANDIDATES_FOUND,
    NO_CANDIDATES,
    PARKED_STAYED,     // the search found nowhere better, so the snapshot still describes where the arm stands
    PARKED_MOVED,      // the vehicle drove somewhere the arm can work from, and the snapshot is now stale
    NO_PARK,           // nowhere in the bounded box brings the target into reach
    PLAN_FOUND,
    NO_PLAN,
    REPROCESSING,      // the fresh frames are in and the cloud node is processing them
    RECANDIDATES_FOUND,
    REPLAN_FOUND,
    REVERIFY_RETRY,    // the fresh look found nothing to grab, or nothing it could plan
    REVERIFY_DUE,
    REACHED,           // the executor reached the grasp
    JAW_SETTLED,       // the jaw closed and stopped moving, or ran out of time
    RETRY_DUE,
    GAVE_UP,           // max_attempts or reverify/attempts used up
    FAILURE            // a node refused or failed
};

const char *stateName(State state);

// The state `event` leads to from `state`; `state` itself when the event does not apply there.
State nextState(State state, Event event);

}  // namespace task

#endif  // TASK_STATEMACHINE_H
