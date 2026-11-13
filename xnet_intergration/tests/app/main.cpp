#include "runner.hpp"

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        usage(std::cout);
        return EXIT_SUCCESS;
    }
    try {
        const auto options = parse_options(argc, argv);
        const auto config = read_config(options.config);
        if (std::signal(SIGINT, request_shutdown) == SIG_ERR || std::signal(SIGTERM, request_shutdown) == SIG_ERR)
            throw std::runtime_error("cannot install shutdown signal handlers");
#ifndef _WIN32
        // A closed output pipe should fail through normal cleanup, not SIGPIPE.
        if (std::signal(SIGPIPE, SIG_IGN) == SIG_ERR) throw std::runtime_error("cannot ignore SIGPIPE");
#endif
        Library library(options.library);
        const Api api(library);
        Device device(api, library);
        std::cerr << "Library: " << options.library << "\nRunner config: " << options.config
                  << "\nInterface: " << config.interface_name << "\nModel: " << config.model
                  << "\nPersistence: " << config.persistence << (fs::exists(config.persistence) ? " (exists)" : " (not yet created)")
                  << "\nSaved LSS settings may override model startup settings; this app never deletes persistence."
                  << "\nUse a library built with the matching 48-byte status ABI."
                  << "\nNative log location is fixed when the library is built; its directory must be writable."
                  << "\nThis is a passive slave: an external master controls NMT; no application/drive model is supplied.\n";
        bool ok = false;
        try {
            if (!api.check(api.create(&device.handle), "xnet_canopen_create", device.handle))
                throw std::runtime_error("device creation failed");
            if (!shutdown_signal) {
                device.initialize_attempted = true;
                const auto model = config.model.string();
                const auto persistence = config.persistence.string();
                if (!api.check(api.initialize(device.handle, config.interface_name.c_str(), model.c_str(), persistence.c_str()),
                               "xnet_canopen_initialize", device.handle))
                    throw std::runtime_error("device initialization failed");
                device.initialized = true;
                api.diagnostic(device.handle);
                if (!shutdown_signal && !api.check(api.start(device.handle), "xnet_canopen_start", device.handle))
                    throw std::runtime_error("device start failed");
            }
            ok = run(api, device, options.duration_seconds);
        } catch (const std::exception& error) {
            std::cerr << "Error: " << error.what() << '\n';
        }
        const bool clean = device.cleanup();
        const bool unloaded = library.close();
        return ok && clean && unloaded ? EXIT_SUCCESS : EXIT_FAILURE;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        usage(std::cerr);
        return EXIT_FAILURE;
    }
}
