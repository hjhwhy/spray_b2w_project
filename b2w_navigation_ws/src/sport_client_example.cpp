//
// ## 演示用，非正式代码  ##
//
#include <iostream>
#include <string>
#include <chrono>
#include <mutex>
#include <thread>
#include <pthread.h>
#include <stdexcept>
#include <unitree/robot/b2/sport/sport_client.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

using namespace std;

struct TestOption
{
    std::string name; 
    int id;           
};

const vector<TestOption> option_list =
    {{"damp", 0},        
     {"stand_up", 1},    
     {"stand_down", 2},  
     {"move forward", 3},
     {"move forward_seconds", 30},
     {"move_backward", 31},
     {"move_backward_seconds", 32},
     {"move lateral", 4}, 
     {"move lateral_seconds", 40}, 
     {"move_right_lateral", 41},
     {"move_right_lateral_seconds", 42},       
     {"move rotate", 5},   
     {"move rotate_5_seconds", 50},
     {"move_rotate_clockwise", 51},
     {"move_rotate_clockwise_seconds", 52},
     {"stop_move", 6},   
     {"switch_gait", 7}, 
     {"switch_gait", 8},
     {"recovery", 9},   
     {"euler_roll", 10},
    }; 

// === 新增：加载 YAML 配置 ===
struct MoveParams {
    double forward = 0.4;
    double lateral = 0.3;
    double rotate = 0.5;
    double forward_timed = 0.3;
    double lateral_timed = 0.3;
    double rotate_timed = 0.3;
};

MoveParams loadMoveParams(const std::string& config_path) {
    MoveParams params;
    try {
        if (!std::filesystem::exists(config_path)) {
            std::cerr << "Config file not found: " << config_path << std::endl;
            return params;
        }
        YAML::Node config = YAML::LoadFile(config_path);
        if (config["move"]) {
            auto m = config["move"];
            params.forward = m["forward_speed"].as<double>(params.forward);
            params.lateral = m["lateral_speed"].as<double>(params.lateral);
            params.rotate = m["rotate_speed"].as<double>(params.rotate);
        }
        if (config["move_timed"]) {
            auto mt = config["move_timed"];
            params.forward_timed = mt["forward_time"].as<double>(params.forward_timed);
            params.lateral_timed = mt["lateral_time"].as<double>(params.lateral_timed);
            params.rotate_timed = mt["rotate_time"].as<double>(params.rotate_timed);
        }
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
    }
    return params;
}

int ConvertToInt(const std::string &str)
{
    try
    {
        std::stoi(str);
        return std::stoi(str);
    }
    catch (const std::invalid_argument &)
    {
        return -1;
    }
    catch (const std::out_of_range &)
    {
        return -1; 
    }
}

class UserInterface
{
public:
    UserInterface(){};
    ~UserInterface(){};

    void terminalHandle()
    {
        std::string input;
        std::getline(std::cin, input);
        if (input.compare("list") == 0)
        {
            for (TestOption option : option_list) {
                std::cout << option.name << ", id: " << option.id << std::endl;
            }
        }
        for (TestOption option : option_list)
        {
            if (input.compare(option.name) == 0 || ConvertToInt(input) == option.id)
            {
                test_option_->id = option.id;
                test_option_->name = option.name;
                std::cout << "Test: " << test_option_->name << ", test_id: " << test_option_->id << std::endl;
            }
        }
    };

    TestOption *test_option_;
};

const TestOption* getOptionById(int id)
{
    for (const auto& opt : option_list)
        if (opt.id == id)
            return &opt;
    return nullptr;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    std::string config_file = "config/sport_client_example_params.yaml";
    if (argc >= 3) {
        config_file = argv[2];
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
    // 加载参数
    MoveParams move_params = loadMoveParams(config_file);

    TestOption test_option;
    test_option.id = 1;

    unitree::robot::b2::SportClient sport_client;
    sport_client.SetTimeout(25.0f);
    sport_client.Init();

    UserInterface user_interface;
    user_interface.test_option_ = &test_option;
    std::cout << "Input \"list \" to list all test option ..." << std::endl;
    long res_count = 0;
    while (1)
    {
        auto time_start_trick = std::chrono::high_resolution_clock::now();
        static const constexpr auto dt = std::chrono::microseconds(20000); // 50Hz
        user_interface.terminalHandle();

        int res = 1;
        if (test_option.id == 0) {
            res = sport_client.Damp();
        }
        else if (test_option.id == 1) {
            res = sport_client.StandUp();
        }
        else if (test_option.id == 2) {
            res = sport_client.StandDown();
        }
        else if (test_option.id == 3) {
            // the robot will move for 0.5 seconds before stopping.
            // If the Move command is called in a loop, the robot will continue moving. 
            // If no Move request is received within 0.5 seconds, the robot will automatically stop.
            res = sport_client.Move(move_params.forward, 0, 0);
        }
        else if (test_option.id == 31) {
            res = sport_client.Move(-move_params.forward, 0, 0);
        }
        else if (test_option.id == 30) {
            const double duration = move_params.forward_timed;  
            const double dt_temp = 0.2;        // 每 200ms 发一次指令
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(move_params.forward, 0.0, 0.0); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 32) {
            const double duration = move_params.forward_timed;  
            const double dt_temp = 0.2;      
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(-move_params.forward, 0.0, 0.0); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 4) {
            res = sport_client.Move(0, move_params.lateral, 0);
        }
        else if (test_option.id == 41) {
            res = sport_client.Move(0, -move_params.lateral, 0);
        }
        else if (test_option.id == 40) {
            const double duration = move_params.lateral_timed;  
            const double dt_temp = 0.2;       
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(0.0,  move_params.lateral, 0.0); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 42) {
            const double duration = move_params.lateral_timed;  
            const double dt_temp = 0.2;       
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(0.0,  -move_params.lateral, 0.0); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 5) {
            res = sport_client.Move(0, 0, move_params.rotate);
        }
        else if (test_option.id == 51) {
            res = sport_client.Move(0, 0, -move_params.rotate);
        }
        else if (test_option.id == 50) {
            const double duration = move_params.rotate_timed;  
            const double dt_temp = 0.2;        
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(0.0, 0.0, move_params.rotate); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 52) {
            const double duration = move_params.rotate_timed;  
            const double dt_temp = 0.2;        
            const auto start_time = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - start_time < std::chrono::duration<double>(duration)) {
                res = sport_client.Move(0.0, 0.0, -move_params.rotate); 
                std::this_thread::sleep_for(std::chrono::duration<double>(dt_temp));
            }
            sport_client.Move(0.0, 0.0, 0.0);
        }
        else if (test_option.id == 6) {
            res = sport_client.StopMove();
        }
        else if (test_option.id == 7) {
            res = sport_client.SwitchGait(0);
        }
        else if (test_option.id == 8) {
            res = sport_client.SwitchGait(1);
        }
        else if (test_option.id == 9) {
            res = sport_client.RecoveryStand();
        }
        else if (test_option.id == 10) {
            res = sport_client.Euler(0.6, 0, 0);
        }
        
        const TestOption* opt = getOptionById(test_option.id);
        if (res < 0) {
            res_count += 1;
            if (opt)
                std::cout << "Request error for: " << opt->name << ", code: " << res << ", count: " << res_count << std::endl;
            else 
                std::cout << "Request error, id = " << test_option.id << std::endl;
        }
        else {
            res_count = 0;
            if (opt)
                std::cout << "Request successed: " << opt->name << ", code: " << res << std::endl;
            else 
                std::cout << "Request successed, id = " << test_option.id << std::endl;
        }
        std::this_thread::sleep_until(time_start_trick + dt);
    }
    return 0;
}