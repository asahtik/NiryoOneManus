#include "manus_node/manus_node.h"

NiryoOneManipulator::NiryoOneManipulator(const NiryoOneManusInterface* mi, const string& device, const string& model, const string& servos) {
    
}

NiryoOneManipulator::~NiryoOneManipulator() {

}

int NiryoOneManipulator::size() {

}

bool NiryoOneManipulator::move(int joint, float position, float speed) {

}

ManipulatorDescription NiryoOneManipulator::describe() {

}

ManipulatorState NiryoOneManipulator::state() {
    
}

void btnPressedISR() {
    btnPressState = true;
}

int main(int argc, char** argv) {
    OUTPUT_INFO("Starting up");

    wiringPiSetupGpio();

    change_led(false, false, true);

    auto mi = std::shared_ptr(new NiryoOneManusInterface());
    mi->init();

    pinMode(BTN_PIN, INPUT);
    pullUpDnControl(BTN_PIN, PUD_UP);
    wiringPiISR(BTN_PIN, INT_EDGE_FALLING, &btnPressedISR);

    OUTPUT_INFO("Waiting for start of calibration");
    while (!btnPressState) repl::sleep(0.5);

    mi->calibrate();

    change_led(false, true, false);

    // TODO: while loop

    // TODO: on exit go to 0 and disable torque
}


    

    

    while (ok) {

    }