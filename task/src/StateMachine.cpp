// Copyright by BeeX [2026]

#include <task/StateMachine.h>

namespace task {

const char *stateName(State state) {
    switch (state) {
    case State::IDLE:
        return "IDLE";
    case State::COLLECT:
        return "COLLECT";
    case State::PROCESS:
        return "PROCESS";
    case State::PARK:
        return "PARK";
    case State::PLAN:
        return "PLAN";
    case State::RECOLLECT:
        return "RECOLLECT";
    case State::REPROCESS:
        return "REPROCESS";
    case State::REPLAN:
        return "REPLAN";
    case State::REVERIFY:
        return "REVERIFY";
    case State::EXECUTE:
        return "EXECUTE";
    case State::JAWCLOSING:
        return "JAWCLOSING";
    case State::DONE:
        return "DONE";
    case State::RETRY:
        return "RETRY";
    case State::FAILED:
        return "FAILED";
    case State::STOPPED:
        return "STOPPED";
    }
    return "UNKNOWN";
}

State nextState(State state, Event event) {
    const bool working = state == State::COLLECT || state == State::PROCESS || state == State::PARK
                         || state == State::PLAN || state == State::RECOLLECT || state == State::REPROCESS
                         || state == State::REPLAN || state == State::EXECUTE || state == State::JAWCLOSING;
    const bool finished = state == State::IDLE || state == State::DONE || state == State::FAILED
                          || state == State::STOPPED;

    switch (event) {
    case Event::NONE:
        return state;
    case Event::START:
        return finished ? State::COLLECT : state;
    case Event::STOP:
        return State::STOPPED;
    case Event::PROCESSING:
        return state == State::COLLECT ? State::PROCESS : state;
    case Event::CANDIDATES_FOUND:
        return state == State::COLLECT || state == State::PROCESS ? State::PARK : state;
    case Event::PARKED_STAYED:
        return state == State::PARK ? State::PLAN : state;
    case Event::PARKED_MOVED:
        return state == State::PARK ? State::RECOLLECT : state;
    case Event::NO_PARK:
        return state == State::PARK ? State::RETRY : state;
    case Event::NO_CANDIDATES:
        return state == State::COLLECT || state == State::PROCESS ? State::RETRY : state;
    case Event::PLAN_FOUND:
        return state == State::PLAN ? State::EXECUTE : state;
    case Event::NO_PLAN:
        return state == State::PLAN ? State::RETRY : state;
    case Event::REPROCESSING:
        return state == State::RECOLLECT ? State::REPROCESS : state;
    case Event::RECANDIDATES_FOUND:
        return state == State::RECOLLECT || state == State::REPROCESS ? State::REPLAN : state;
    case Event::REPLAN_FOUND:
        return state == State::REPLAN ? State::EXECUTE : state;
    case Event::REVERIFY_RETRY:
        return state == State::RECOLLECT || state == State::REPROCESS || state == State::REPLAN ? State::REVERIFY
                                                                                                : state;
    case Event::REVERIFY_DUE:
        return state == State::REVERIFY ? State::RECOLLECT : state;
    case Event::REACHED:
        return state == State::EXECUTE ? State::JAWCLOSING : state;
    case Event::JAW_SETTLED:
        return state == State::JAWCLOSING ? State::DONE : state;
    case Event::RETRY_DUE:
        return state == State::RETRY ? State::COLLECT : state;
    case Event::GAVE_UP:
        return state == State::RETRY || state == State::REVERIFY ? State::FAILED : state;
    case Event::FAILURE:
        return working ? State::FAILED : state;
    }
    return state;
}

}  // namespace task
