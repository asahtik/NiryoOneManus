#include <signal.h>
#include "manus_node/manus_node.h"

std::string MODEL_PATH;

volatile bool calibrationRequested = false;
volatile bool calibrated = false;

void rwCtrlLoop(std::shared_ptr<NiryoOneManusInterface> i) {
    repl::Rate r(100);
    while (!shuttingDown) {
        i->read();
        if (calibrated) {   
            i->write();
        } else if (calibrationRequested) {
            mi->calibrate();
            mi->openGripper(1.0);
            repl::sleep(2);
            calibrated = true;
            change_led(false, true, false);
        }
        r.sleep();
    }
}

void NiryoOneManipulator::loadDescription() {
    try {
        parse_description(MODEL_PATH, mDescription);
    } catch(...) {
        OUTPUT_ERROR("Error parsing description");
        mi->shutdown();
        change_led(true, false, true);
        exit(1);
    }

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

int jointToCmd(int joint, ManipulatorDescription& desc) {
    int cmd = joint;
    for (int i = joint; i >= 0; --i) {
        if (desc.joints.at(i).type == JOINTTYPE_FIXED) --cmd;
    }
    return cmd;
}

bool NiryoOneManipulator::move(int joint, float position, float speed) {
    if (!shuttingDown) {
        OUTPUT_INFO("Joint: %d - Position: %f", joint, position);
        auto jointD = mDescription.joints.at(joint);
        if (jointD.type != JOINTTYPE_GRIPPER && jointD.type != JOINTTYPE_FIXED)
            mi->cmd[jointToCmd(joint, mDescription)] = normalisePosition(jointD, position);
        else if (jointD.type != JOINTTYPE_GRIPPER) {
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
    int cmd = 0;
    for (unsigned int i = 0; i < noJoints - 1; ++i) {
        if (mDescription.joints.at(i).type != JOINTTYPE_FIXED) {
            mState.joints.at(i).position = mi->pos[cmd];
            mState.joints.at(i).goal = mi->cmd[cmd];
            mState.joints.at(i).speed = mi->vel[cmd];
            ++cmd;
        } else {
            mState.joints.at(i).position = 0;
            mState.joints.at(i).goal = 0;
            mState.joints.at(i).speed = 0;
        }
    }
    mState.joints.at(noJoints - 1).position = !mi->gripperPos;
    mState.joints.at(noJoints - 1).goal = mi->gripperCmd;
    mState.joints.at(noJoints - 1).speed = mi->gripperVel;
    mState.header.timestamp = system_clock::now();
    return mState;
}

void NiryoOneManipulator::prepareNewGoal(bool begin_trajectory = false) {
    mi->syncNextGoal(true);
}

void shutdown(bool shutdownSystem) {
    shuttingDown = true;
    change_led(true, false, true);
    mi->shutdown();
    if (shutdownSystem)
        system("shutdown 0");
    else exit(0);
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
            shutdown(true);
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

void sigintSR(int s) {
    OUTPUT_WARNING("Shutting down program");
    shutdown(false);
}

void setupSigint() {
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = sigintSR;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGSTOP, &sigIntHandler, NULL);
    sigaction(SIGABRT, &sigIntHandler, NULL);
}

bool torqueOn = true;
void sigusr1SR(int s) {
    OUTPUT_WARNING("Starting learning mode %d", torqueOn);
    torqueOn = !torqueOn;
    mi->activateTorque(torqueOn);
}

void setupSigusr() {
    struct sigaction sigUsrHandler;
    sigUsrHandler.sa_handler = sigusr1SR;
    sigemptyset(&sigUsrHandler.sa_mask);
    sigUsrHandler.sa_flags = 0;
    sigaction(SIGUSR1, &sigUsrHandler, NULL);
}

int main(int argc, char** argv) {
    if (argc > 1)
        MODEL_PATH = argv[1];
    else
        MODEL_PATH =  getenv("MODEL_DESCRIPTION");
    OUTPUT_INFO("Model path: %s", MODEL_PATH.c_str());
    OUTPUT_INFO("Starting up");
    setupGpio();
    last_pressed = repl::time_now();

    change_led(false, false, true);

    mi.reset(new NiryoOneManusInterface());
    mi->init();

    attachBtnInterrupt();
    setupSigint();

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