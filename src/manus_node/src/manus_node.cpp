#include "manus_node/manus_node.h"

void rwCtrlLoop(std::shared_ptr<NiryoOneManusInterface> i, bool& notOk) {
    repl::Rate r(100);
    while (!notOk) {
        i->read();
        i->write();
        r.sleep();
    }
}

NiryoOneManipulator::NiryoOneManipulator() {
    loadDescription();
    rwThread.reset(new std::thread(&rwCtrlLoop, mi, std::ref(shuttingDown)));
}

NiryoOneManipulator::~NiryoOneManipulator() {
    shutdown();
}

int NiryoOneManipulator::size() {
    return joints.size();
}

bool NiryoOneManipulator::move(int joint, float position, float speed) {
    if (!shuttingDown) mi->cmd[joint] = position;
    return true;
}

// ManipulatorDescription NiryoOneManipulator::describe() {
//     // TODO: Not yet implemented
// }

// ManipulatorState NiryoOneManipulator::state() {
//     // TODO: Not yet implemented
// }

void NiryoOneManipulator::loadDescription() {
    parse_description("model.yaml", joints);
}

void NiryoOneManipulatorManager::step(bool force) {
    if (!plan) return;

    bool idle = true;
    bool goal = true;

    ManipulatorState state = manipulator->state();
    for (int i = 0; i < manipulator->size(); i++) {
        idle &= state.joints[i].type == JOINTSTATETYPE_IDLE;
        goal &= close_enough(state.joints[i].position, state.joints[i].goal);
    }

    if (goal || force) {

        if (plan->segments.size() == 0) {
            PlanState state;
            state.identifier = plan->identifier;
            state.type = PLANSTATETYPE_COMPLETED;
            planstate_publisher->send(state);
            plan.reset();

            lastPlanSize = 0;
            return;
        }

        mi->syncNextGoal(lastPlanSize == 0);
        for (int i = 0; i < manipulator->size(); i++) {
            manipulator->move(i, plan->segments[0].joints[i].goal, plan->segments[0].joints[i].speed);
        }

        plan->segments.erase(plan->segments.begin());
    }
}

void shutdown() {
    shuttingDown = true;
    change_led(true, false, true);
    mi->shutdown();
    system("shutdown 0");
}

repl::Time last_pressed;
void btnStateSwitchISR() {
    bool btn = !(bool)digitalRead(BTN_PIN);
    auto now = repl::time_now();
    if (btn && noBtnPresses == 0) {
        last_pressed = now;
        btnPressed = true;
    } else if (btn && (now - last_pressed) > repl::Millis(500)) {
        last_pressed = now;
        btnPressed = true;
    } else if (!btn && (now - last_pressed) > repl::Millis(100)) {
        btnPressed = false;
        if ((now - last_pressed) > repl::Millis(5000)) {
            shutdown();
        } else if (noBtnPresses == 0) calibrationRequested = true;
        ++noBtnPresses;
    }
}

int main(int argc, char** argv) {
    OUTPUT_INFO("Starting up");
    
    last_pressed = repl::time_now();
    wiringPiSetupGpio();

    change_led(false, false, true);

    mi.reset(new NiryoOneManusInterface());
    mi->init();

    pinMode(BTN_PIN, INPUT);
    pullUpDnControl(BTN_PIN, PUD_UP);
    wiringPiISR(BTN_PIN, INT_EDGE_FALLING, &btnStateSwitchISR);

    OUTPUT_INFO("Waiting for start of calibration");
    while (!calibrationRequested) repl::sleep(0.5);

    mi->calibrate();

    change_led(false, true, false);

    try {

        std::shared_ptr<NiryoOneManipulator> manipulator(new NiryoOneManipulator());

        SharedClient client = echolib::connect(string(), "manipulator");
        ManipulatorManager manager(client, manipulator);

        int duration = 0;

        bool ok = true;
        while (ok) {
            if (!echolib::wait(std::max(1, 20 - duration))) break;

            steady_clock::time_point start = steady_clock::now();

            manager.update();

            duration = duration_cast<milliseconds>(steady_clock::now() - start).count();
        }
    } catch (ManipulatorException &e) {
        cout << "Exception: " << e.what() << endl;
        shutdown();
        exit(1);
    }

    shutdown();
    exit(0);
}