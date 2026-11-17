/**
 * @file Dynamically loaded CANopen slave with an external simulated motor.
 * @copyright 2026 CANopenNode contributors
 */
#include "../app/runner.hpp"
#include "simulator.hpp"

namespace {
void modsim_usage(std::ostream& out) {
    out << "Usage: xnet_canopen_modsim --lib <shared-library> --config <runner.json>"
           " [--motor <motor.json>] [--duration <positive-integer-seconds>]\n"
           "External single-axis PP slave; master controls NMT and motion. Software only.\n";
}
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        modsim_usage(std::cout);
        return EXIT_SUCCESS;
    }
    try {
        const auto options = parse_options(argc, argv, true);
        const auto config = read_config(options.config);
        const auto parameters = modsim::read_parameters(options.motor);
        std::ifstream model_file(config.model);
        if (!model_file) throw std::runtime_error("cannot open CANopen model");
        const auto model_json = nlohmann::json::parse(model_file);
        if (!model_json.contains("drive_402") || model_json.at("drive_402").at("model") != "external"
            || model_json.at("drive_402").at("profile") != "PP")
            throw std::runtime_error("modsim requires drive_402.model=external and profile=PP");
        if (std::signal(SIGINT, request_shutdown) == SIG_ERR || std::signal(SIGTERM, request_shutdown) == SIG_ERR)
            throw std::runtime_error("cannot install signal handlers");
#ifndef _WIN32
        if (std::signal(SIGPIPE, SIG_IGN) == SIG_ERR) throw std::runtime_error("cannot ignore SIGPIPE");
#endif
        Library library(options.library);
        const Api api(library);
        const modsim::ExchangeApi exchange{
            library.resolve<decltype(&xnet_canopen_get_motor_drive_endpoint)>("xnet_canopen_get_motor_drive_endpoint"),
            library.resolve<decltype(&motor_drive_open_v1)>("motor_drive_open_v1"),
            library.resolve<decltype(&motor_drive_read_v1)>("motor_drive_read_v1"),
            library.resolve<decltype(&motor_drive_submit_v1)>("motor_drive_submit_v1"),
            library.resolve<decltype(&motor_drive_heartbeat_v1)>("motor_drive_heartbeat_v1"),
            library.resolve<decltype(&motor_drive_close_v1)>("motor_drive_close_v1"),
            api.stop, api.get_last_error};
        Device device(api, library);
        bool ok = false;
        try {
            if (!api.check(api.create(&device.handle), "create", device.handle))
                throw std::runtime_error("device creation failed");
            device.initialize_attempted = true;
            if (!api.check(api.initialize(device.handle, config.interface_name.c_str(), config.model.string().c_str(),
                    config.persistence.string().c_str()), "initialize", device.handle))
                throw std::runtime_error("device initialization failed");
            device.initialized = true;
            modsim::Simulator simulator(exchange, device.handle, parameters);
            if (!api.check(api.start(device.handle), "start", device.handle))
                throw std::runtime_error("device start failed");
            simulator.start();
            std::cerr << "External PP slave: interface=" << config.interface_name << " model=" << config.model
                      << " persistence=" << config.persistence << " period_us=" << parameters.period_us
                      << "\nSoftware simulation only; use exclusive interface/persistence access."
                      << "\nSaved LSS settings may override node/bitrate. Native log path is fixed at library build time.\n";
            const auto begin = Clock::now();
            auto next = begin;
            while (!shutdown_signal && (!options.duration_seconds
                    || Clock::now() - begin < std::chrono::seconds(options.duration_seconds))) {
                if (simulator.failed()) throw std::runtime_error("external model thread failed");
                if (Clock::now() >= next) {
                    xnet_canopen_status_t status{};
                    if (!api.check(api.get_status(device.handle, &status), "get_status", device.handle)
                        || status.runtime_state != XNET_CO_RUNTIME_RUNNING)
                        throw std::runtime_error("native runtime is no longer running");
                    print_status(status);
                    const auto state = simulator.snapshot();
                    std::cout << "modsim epoch=" << state.command.token.epoch
                              << " action=" << state.command.action << " action_seq=" << state.command.token.action_seq
                              << " demand_rad=" << state.command.demand_position_rad
                              << " actual_rad=" << state.feedback.actual_position_rad
                              << " velocity_rad_s=" << state.feedback.actual_velocity_rad_s
                              << " fault_class=" << state.feedback.fault_class << " accepted=" << state.accepted
                              << " rejected=" << state.rejected << " overruns=" << state.overruns << std::endl;
                    if (!std::cout) throw std::runtime_error("cannot write model status");
                    next = Clock::now() + std::chrono::milliseconds(500);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            if (shutdown_signal) std::cerr << "Shutdown requested by signal " << shutdown_signal << ".\n";
            ok = simulator.stop();
            const auto state = simulator.snapshot();
            std::cerr << "Model stopped: accepted=" << state.accepted << " rejected=" << state.rejected
                      << " epochs=" << state.epochs << '\n';
            if (state.accepted == 0) {
                std::cerr << "No external feedback was accepted.\n";
                ok = false;
            }
        } catch (const std::exception& error) {
            std::cerr << "Error: " << error.what() << '\n';
            api.diagnostic(device.handle);
        }
        const bool clean = device.cleanup();
        const bool unloaded = library.close();
        return ok && clean && unloaded ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        modsim_usage(std::cerr);
        return EXIT_FAILURE;
    }
}
