// to retrive data from the pre-recorded .rec files

// rec2csv.cpp
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    std::string inputFile = "recording.rec"; // default
    std::string outputFile = "outputs/steering.csv";

    // Optional: Parse input/output filenames from command line
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--input") inputFile = argv[++i];
        if (std::string(argv[i]) == "--output") outputFile = argv[++i];
    }

    // Open CSV output file
    std::ofstream out(outputFile);
    if (!out.is_open()) {
        std::cerr << "Could not open output file: " << outputFile << std::endl;
        return 1;
    }
    out << "timestamp;groundSteeringAngle\n";

    // Setup OD4Session for replaying the .rec file
    cluon::OD4Session replaySession({0, 0, "0.0.0.0", 0}, inputFile);

    // Listen for ground steering messages
    replaySession.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
        [&out](cluon::data::Envelope &&env) {
            auto gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            uint64_t timestamp = env.sampleTimeStamp().microseconds();
            float angle = gsr.groundSteering();
            out << timestamp << ";" << angle << "\n";
        });

    // Let the replay run until the file ends
    while (replaySession.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    out.close();
    return 0;
}
