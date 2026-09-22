// Copyright by BeeX [2026]

#include <task/StateMachine.h>

namespace task {

const char *stateName(State state) {
    switch (state) {
    case State::READY:
        return "READY";
    case State::STREAM:
        return "STREAM";
    case State::COLLECT:
        return "COLLECT";
    case State::PROCESS:
        return "PROCESS";
    case State::PICKSPOT:
        return "PICKSPOT";
    case State::GOTOSPOT:
        return "GOTOSPOT";
    case State::RESURVEY:
        return "RESURVEY";
    case State::PICKGRASP:
        return "PICKGRASP";
    case State::GOTOGRASP:
        return "GOTOGRASP";
    case State::CLOSEJAW:
        return "CLOSEJAW";
    case State::RETARGET:
        return "RETARGET";
    case State::REPARK:
        return "REPARK";
    case State::SUCCESS:
        return "SUCCESS";
    case State::FAIL:
        return "FAIL";
    case State::ESTOP:
        return "ESTOP";
    }
    return "UNKNOWN";
}

bool isWorking(State state) {
    switch (state) {
    case State::READY:
    case State::SUCCESS:
    case State::FAIL:
    case State::ESTOP:
        return false;
    default:
        return true;
    }
}

State nextState(State state, Event event) {
    // Contact and a human asking for a stop both land in ESTOP: the arm is released either way
    // and neither is something the machine may carry on from by itself. Leaving ESTOP takes
    // another task/start, which is the acknowledgement.
    if (event == Event::COLLIDED || event == Event::STOP) {
        return isWorking(state) ? State::ESTOP : state;
    }
    if (event == Event::FAILURE) {
        return isWorking(state) ? State::FAIL : state;
    }
    if (event == Event::START) {
        return isWorking(state) ? state : State::STREAM;
    }

    switch (state) {
    case State::STREAM:
        if (event == Event::STREAMING) {
            return State::COLLECT;
        }
        return event == Event::NO_STREAM ? State::FAIL : state;

    // The window is still filling on COLLECT and already filled on PROCESS, but the cloud node
    // answers the same way from both, so both take the same verdicts.
    case State::COLLECT:
    case State::PROCESS:
        if (event == Event::PROCESSING) {
            return state == State::COLLECT ? State::PROCESS : state;
        }
        if (event == Event::CANDIDATES_SPOT) {
            return State::PICKSPOT;
        }
        if (event == Event::CANDIDATES_GRASP) {
            return State::PICKGRASP;
        }
        // Nothing in view to park for costs no park: the vehicle has not moved and nothing has
        // been decided, so this loops freely until the survey budget runs out.
        if (event == Event::NO_CANDIDATES_SPOT) {
            return State::RESURVEY;
        }
        return event == Event::NO_CANDIDATES_GRASP ? State::RETARGET : state;

    case State::RESURVEY:
        if (event == Event::SURVEY_AGAIN) {
            return State::COLLECT;
        }
        return event == Event::OUT_OF_TIME ? State::FAIL : state;

    case State::PICKSPOT:
        if (event == Event::SPOT_CHOSEN) {
            return State::GOTOSPOT;
        }
        // Standing still still earns a second look: the frames behind the decision were taken
        // before it, and the grasp is planned from what is there now.
        if (event == Event::SPOT_UNCHANGED) {
            return State::COLLECT;
        }
        return event == Event::NO_SPOT ? State::REPARK : state;

    case State::GOTOSPOT:
        return event == Event::ARRIVED ? State::COLLECT : state;

    case State::PICKGRASP:
        if (event == Event::PLAN_FOUND) {
            return State::GOTOGRASP;
        }
        return event == Event::NO_PLAN ? State::RETARGET : state;

    // Running out of arrival time is not contact: the path was clear when it was planned and
    // nothing reported a joint against a stop, so it is worth another look rather than a latch.
    case State::GOTOGRASP:
        if (event == Event::REACHED) {
            return State::CLOSEJAW;
        }
        return event == Event::STALLED ? State::RETARGET : state;

    // An empty jaw is recorded, not punished. There is no handle to catch in simulation, and
    // the grip thresholds have never been checked on the bench.
    case State::CLOSEJAW:
        return event == Event::JAW_SETTLED ? State::SUCCESS : state;

    case State::RETARGET:
        if (event == Event::LOOK_AGAIN) {
            return State::COLLECT;
        }
        return event == Event::OUT_OF_LOOKS ? State::REPARK : state;

    case State::REPARK:
        if (event == Event::PARK_AGAIN) {
            return State::COLLECT;
        }
        return event == Event::OUT_OF_PARKS ? State::FAIL : state;

    default:
        return state;
    }
}

}  // namespace task
