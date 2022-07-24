#include <memory>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <echolib/client.h>
#include <echolib/helpers.h>
#include <manus/messages.h>

#define GRIP_WAIT 5
#define MOVE_WAIT 5
#define TARGET_WAIT 5

#define HOVER_HEIGHT 100
#define PICKUP_Y_ANGLE -1.0

using namespace echolib;
using namespace manus::messages;

class Manager {
private:
    SharedClient client;

    SharedTypedSubscriber<ManipulatorDescription> description_subscriber;
	SharedTypedSubscriber<ManipulatorState> state_subscriber;

    ManipulatorDescription description;
    ManipulatorState state;

    bool description_ready = false;
    bool state_ready = false;
protected:
    void on_description(shared_ptr<ManipulatorDescription> desc) {
        description = *desc;
        description_ready = true;
    }
    void on_state(shared_ptr<ManipulatorState> st) {
        state = *st;
        state_ready = true;
    }
public:
    SharedTypedPublisher<Trajectory> trajectory_publisher;
    SharedTypedPublisher<Plan> plan_publisher;

    Manager(SharedClient client) : client(client) {
        description_subscriber = make_shared<TypedSubscriber<ManipulatorDescription>>(client, "manipulator0.description", bind(&Manager::on_description, this, placeholders::_1));
        state_subscriber = make_shared<TypedSubscriber<ManipulatorState>>(client, "manipulator0.state", bind(&Manager::on_state, this, placeholders::_1));
        trajectory_publisher = make_shared<TypedPublisher<Trajectory>>(client, "manipulator0.trajectory");
        plan_publisher = make_shared<TypedPublisher<Plan>>(client, "manipulator0.plan");
    }
    PlanSegment state_to_segment() {
        PlanSegment ret;
        for (auto j : this->state.joints)
            ret.joints.emplace_back(j.goal, j.speed);
        return ret;
    }
    bool is_ready() {
        return description_ready && state_ready;
    }
    bool is_idle() {
        for (auto j : state.joints) {
            if (j.type != JointStateType::JOINTSTATETYPE_IDLE) return false;
        }
        return true;
    }
};

vector<tuple<float, float>> points;
void parse_points(const std::string& filename) {
    points.clear();
    points.emplace_back(0.0, 0.0);
    try {
        YAML::Node doc = YAML::LoadFile(filename);
        auto points_node = doc["points"];
        if (points.size() == 0) throw "No points";
        for (unsigned int i = 0; i < points.size(); ++i) 
            points.emplace_back(points_node[i]["x"].as<float>(), points_node[i]["y"].as<float>());
        cout << "Parsed joints" << endl;
    } catch (...) {
        points.emplace_back(0.0, -150.0);
        points.emplace_back(0.0, 150.0);
        points.emplace_back(150.0, -150.0);
        points.emplace_back(150.0, 150.0);
        cerr << "Error reading file" << endl;
    }
}

int main(int argc, char** argv) {
    cout << "Program started" << endl;
    parse_points(argc > 1 ? argv[1] : "");
    SharedClient client = connect(string(), "tester");
    auto manager = make_shared<Manager>(client);
    while (!manager->is_ready() && echolib::wait(500));
    cout << "Program initialised" << endl;
    auto handler = std::thread([]() {
        while (true) {
            echolib::wait(100);
        }
    });
    unsigned int plan = 0;
    unsigned int trajectory = 0;
    unsigned int gripped = 1;
    while (true) {
        while (!manager->is_idle())
            this_thread::sleep_for(chrono::milliseconds(500));
        int wait_cnt = 0;
        cout << "Input goal number: " << endl;
        unsigned int goal = 0;
        cin >> goal;
        if (goal > points.size()) {
            cout << "Invalid goal number." << endl;
            continue;
        } else if (goal == 0) {
            wait_cnt = 0;
            while (wait_cnt <= GRIP_WAIT) {
                cout << "\rTime until " << (gripped ? "release" : "grip") << ": " << GRIP_WAIT - wait_cnt << flush;
                this_thread::sleep_for(chrono::milliseconds(1000));
                ++wait_cnt;
            }
            cout << "\r" << (gripped ? "Releasing" : "Gripping") << endl;
            PlanSegment grip = manager->state_to_segment();
            grip.joints.at(grip.joints.size() - 1).goal = gripped;
            grip.joints.at(grip.joints.size() - 1).speed = 1.0;
            Plan grip_plan;
            grip_plan.identifier = "plan" + to_string(plan++);
            grip_plan.segments.push_back(grip);
            manager->plan_publisher->send(grip_plan);
            gripped = !gripped;
            continue;
        }
        --goal;

        wait_cnt = 0;
        while (wait_cnt <= MOVE_WAIT) {
            cout << "\rTime until move: " << MOVE_WAIT - wait_cnt << flush;
            this_thread::sleep_for(chrono::milliseconds(1000));
            ++wait_cnt;
        }

        cout << "\rMoving to " << goal << endl;
        TrajectorySegment hover;
        hover.frame.origin.x = std::get<0>(points[goal]);
        hover.frame.origin.y = std::get<1>(points[goal]);
        hover.frame.origin.z = HOVER_HEIGHT;
        hover.frame.rotation.x = 0.0;
        hover.frame.rotation.y = PICKUP_Y_ANGLE;
        hover.frame.rotation.z = 0.0;
        hover.rotation = true;
        hover.required = true;
        hover.speed = 1;
        hover.gripper = 0.0;
        TrajectorySegment pick_up;
        pick_up.frame.origin.x = std::get<0>(points[goal]);
        pick_up.frame.origin.y = std::get<1>(points[goal]);
        pick_up.frame.origin.z = 5.0;
        pick_up.frame.rotation.x = 0.0;
        pick_up.frame.rotation.y = PICKUP_Y_ANGLE;
        pick_up.frame.rotation.z = 0.0;
        pick_up.rotation = true;
        pick_up.required = true;
        pick_up.speed = 1;
        pick_up.gripper = 0.0;
        Trajectory do_move;
        do_move.identifier = "trajectory" + to_string(trajectory++);
        do_move.speed = 1.0;
        do_move.segments.push_back(hover);
        do_move.segments.push_back(pick_up);
        manager->trajectory_publisher->send(do_move);

        while (!manager->is_idle()) this_thread::sleep_for(chrono::milliseconds(500));

        wait_cnt = 0;
        while (wait_cnt <= GRIP_WAIT) {
            cout << "\rTime until release: " << GRIP_WAIT - wait_cnt << flush;
            this_thread::sleep_for(chrono::milliseconds(1000));
            ++wait_cnt;
        }
        cout << "\rReleasing" << endl;
        PlanSegment grip = manager->state_to_segment();
        grip.joints.at(grip.joints.size() - 1).goal = 1.0;
        grip.joints.at(grip.joints.size() - 1).speed = 1.0;
        Plan grip_plan;
        grip_plan.identifier = "plan" + to_string(plan++);
        grip_plan.segments.push_back(grip);
        manager->plan_publisher->send(grip_plan);
        gripped = false;

        wait_cnt = 0;
        while (wait_cnt <= TARGET_WAIT) {
            cout << "\rMoving back in: " << TARGET_WAIT - wait_cnt << flush;
            this_thread::sleep_for(chrono::milliseconds(1000));
            ++wait_cnt;
        }

        cout << "\rMoving home" << endl;
        PlanSegment home = manager->state_to_segment();
        for (unsigned int i = 0; i < home.joints.size(); ++i) {
            home.joints.at(i).goal = 0.0;
            home.joints.at(i).speed = 1.0;
        }
        Plan home_plan;
        home_plan.identifier = "plan" + to_string(plan++);
        home_plan.segments.push_back(home);
        manager->plan_publisher->send(home_plan);
    }
    handler.join();
}