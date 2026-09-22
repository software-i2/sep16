// Copyright by BeeX [2026]

#ifndef TASK_STATEMACHINE_H
#define TASK_STATEMACHINE_H

namespace task {

// COLLECT and PROCESS are entered twice, once to choose a parking spot and once to choose a
// grasp from where the vehicle ended up. Which pass it is lives in the node as a phase, not in
// the state, so the two passes cannot drift apart; the events carry the phase instead, which
// keeps this function a pure table.
//
// Nothing measured before the drive is planned against: the second pass always runs, even when
// the search elects not to move, so the grasp is planned from frames taken where the arm now
// stands.
enum class State {
    READY,      // nodes are up, waiting for task/start
    STREAM,     // waiting for the camera to produce
    COLLECT,    // gathering a window of frames
    PROCESS,    // turning them into an obstacle map and handle candidates
    PICKSPOT,   // searching for somewhere to park
    GOTOSPOT,   // driving there
    RESURVEY,   // nothing worth parking for was in view; survey again until the budget runs out
    PICKGRASP,  // planning a path to a handle
    GOTOGRASP,  // following it
    CLOSEJAW,   // closing on the handle and reading what was caught
    RETARGET,   // the look from the park pose gave nothing to grab; look again
    REPARK,     // looking again is used up; park somewhere else and start over
    SUCCESS,
    FAIL,
    ESTOP       // something was touched, or a human asked for a stop; the arm is released
};

enum class Event {
    NONE,                 // nothing happened this tick
    START,                // task/start
    STOP,                 // task/stop

    STREAMING,            // frames are arriving from the camera
    NO_STREAM,            // none arrived within the timeout

    PROCESSING,           // the cloud node has its frames and is working on them
    CANDIDATES_SPOT,      // handles found on the pass that chooses where to park
    CANDIDATES_GRASP,     // handles found on the pass that chooses what to grab
    NO_CANDIDATES_SPOT,
    NO_CANDIDATES_GRASP,

    SURVEY_AGAIN,         // still inside the budget for finding something to park for
    OUT_OF_TIME,          // that budget is gone and nothing has ever been in view

    SPOT_CHOSEN,          // the search found somewhere better and the vehicle has to drive
    SPOT_UNCHANGED,       // nowhere beat standing still, so there is nothing to drive
    NO_SPOT,              // nowhere in the box works at all
    ARRIVED,              // the drive finished

    PLAN_FOUND,
    NO_PLAN,
    REACHED,              // the executor reached the grasp
    STALLED,              // it ran out of arrival time without hitting anything

    JAW_SETTLED,          // the jaw stopped moving, or ran out of time; empty is not a failure

    LOOK_AGAIN,           // another look from the same park pose is allowed
    OUT_OF_LOOKS,
    PARK_AGAIN,           // another park is allowed
    OUT_OF_PARKS,

    COLLIDED,             // a joint stopped following its target, so the arm is against something
    FAILURE               // a node refused or died
};

const char *stateName(State state);

// Whether the machine is part way through a pick, as opposed to sitting in READY or finished.
bool isWorking(State state);

// The state `event` leads to from `state`; `state` itself when the event does not apply there.
State nextState(State state, Event event);

}  // namespace task

#endif  // TASK_STATEMACHINE_H
