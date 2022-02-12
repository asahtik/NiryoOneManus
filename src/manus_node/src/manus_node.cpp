#include "manus_node/manus_node.h"

void rwCtrlLoop(std::shared_ptr<NiryoOneManusInterface> i) {
    repl::Rate r(100);
    while (!shuttingDown) {
        i->read();
        i->write();
        r.sleep();
    }
}

void NiryoOneManipulator::loadDescription() {
    parse_description(find_file("niryoonemanipulator.yaml"), mDescription);
    unsigned int noJoints = mDescription.joints.size();
    mState.joints.resize(noJoints);
    mState.state = MANIPULATORSTATETYPE_ACTIVE;

    for (unsigned int i = 0; i < noJoints - 1; ++i) {
        mState.joints.at(i).type = JOINTSTATETYPE_IDLE;
    }
}

NiryoOneManipulator::NiryoOneManipulator() {
    loadDescription();
    rwThread.reset(new std::thread(&rwCtrlLoop, mi));
}

NiryoOneManipulator::~NiryoOneManipulator() {
    shutdown();
}

int NiryoOneManipulator::size() {
    return mDescription.joints.size();
}

double normalisePosition(JointDescription joint, float position) {
    if (position > joint.max) position = joint.max;
    else if (position < joint.min) position = joint.min;
    return position;
}

bool NiryoOneManipulator::move(int joint, float position, float speed) {
    if (!shuttingDown) {
        auto jointD = mDescription.joints.at(joint);
        if (jointD.type != JOINTTYPE_GRIPPER)
            mi->cmd[joint] = normalisePosition(jointD, position);
        else {
            mi->openGripper(normalisePosition(jointD, position));
        }
    }
    return true;
}

ManipulatorDescription NiryoOneManipulator::describe() {
    return mDescription;
}

ManipulatorState NiryoOneManipulator::state() {
    unsigned int noJoints = mDescription.joints.size();
    for (unsigned int i = 0; i < noJoints - 1; ++i) {
        mState.joints.at(i).position = mi->pos[i];
        mState.joints.at(i).goal = mi->cmd[i];
        mState.joints.at(i).speed = mi->vel[i];
    }
    mState.header.timestamp = system_clock::now();
    return mState;
}

void NiryoOneManipulator::prepareNewGoal(bool begin_trajectory = false) {
    mi->syncNextGoal(begin_trajectory);
}

void shutdown() {
    shuttingDown = true;
    change_led(true, false, true);
    mi->shutdown();
    system("shutdown 0");
}

repl::Time last_pressed;
void btnStateSwitchISR() {
    #ifdef __arm__
    bool btn = !(bool)digitalRead(BTN_PIN);
    OUTPUT_INFO("Btn status %d", btn);
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
    #endif
}

void setupGpio() {
    #ifdef __arm__
    wiringPiSetupGpio();
    #endif
}

void attachBtnInterrupt() {
    #ifdef __arm__
    pinMode(BTN_PIN, INPUT);
    pullUpDnControl(BTN_PIN, PUD_UP);
    wiringPiISR(BTN_PIN, INT_EDGE_BOTH, &btnStateSwitchISR);

    OUTPUT_INFO("Waiting for start of calibration");
    #else
    calibrationRequested = true;
    #endif
}

int main(int argc, char** argv) {
    OUTPUT_INFO("Starting up");
    setupGpio();
    last_pressed = repl::time_now();

    change_led(false, false, true);

    mi.reset(new NiryoOneManusInterface());
    mi->init();

    attachBtnInterrupt();

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