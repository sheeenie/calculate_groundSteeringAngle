// rec2csv.cpp
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    std::string inputFile = "data/144821.rec"; // updated default path
    std::string outputFile = "outputs/steering.csv";

    // Optional: Allow overriding via command line
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--input" && i + 1 < argc) inputFile = argv[++i];
        if (std::string(argv[i]) == "--output" && i + 1 < argc) outputFile = argv[++i];
    }

    std::ofstream out(outputFile);
    if (!out.is_open()) {
        std::cerr << "Could not open output file: " << outputFile << std::endl;
        return 1;
    }

    out << "timestamp,groundSteeringAngle\n"; // changed delimiter from ";" to ","

    cluon::OD4Session replaySession({0, 0, "0.0.0.0", 0}, inputFile);

    replaySession.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
        [&out](cluon::data::Envelope &&env) {
            auto gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            uint64_t timestamp = env.sampleTimeStamp().microseconds();
            float angle = gsr.groundSteering();
            out << timestamp << "," << angle << "\n"; // updated delimiter
        });

    while (replaySession.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    out.close();
    return 0;
}
