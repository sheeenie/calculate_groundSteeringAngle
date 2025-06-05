// rec2csv.cpp
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

int main(int argc, char **argv) {
    std::string inputFile = "data/144821.rec";
    std::string outputFile = "outputs/steering.csv";

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--input" && i + 1 < argc) {
            inputFile = argv[++i];
        }
        if (std::string(argv[i]) == "--output" && i + 1 < argc) {
            outputFile = argv[++i];
        }
    }

    std::cout << "Reading from: " << inputFile << std::endl;
    std::cout << "Writing to: " << outputFile << std::endl;

    // Open output file
    std::ofstream out(outputFile);
    if (!out.is_open()) {
        std::cerr << "Error: Could not open output file: " << outputFile << std::endl;
        return 1;
    }

    // Write CSV header
    out << "timestamp,groundSteeringAngle\n";

    // Create replay session
    cluon::OD4Session replaySession({0, 0, "0.0.0.0", 0}, inputFile);
    
    if (!replaySession.isRunning()) {
        std::cerr << "Error: Could not open input file: " << inputFile << std::endl;
        out.close();
        return 1;
    }

    int messageCount = 0;

    // Set up data trigger for GroundSteeringRequest messages
    replaySession.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(),
        [&out, &messageCount](cluon::data::Envelope &&env) {
            auto gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            
            // Convert microseconds to seconds for better readability
            double timestamp = env.sampleTimeStamp().microseconds() / 1000000.0;
            float angle = gsr.groundSteering();
            
            out << std::fixed << std::setprecision(6) << timestamp << "," << angle << "\n";
            messageCount++;
            
            if (messageCount % 100 == 0) {
                std::cout << "Processed " << messageCount << " messages..." << std::endl;
            }
        });

    std::cout << "Starting to process recording..." << std::endl;

    // Process the recording
    while (replaySession.isRunning()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    out.close();
    std::cout << "Finished processing. Total messages: " << messageCount << std::endl;
    std::cout << "Output written to: " << outputFile << std::endl;

    return 0;