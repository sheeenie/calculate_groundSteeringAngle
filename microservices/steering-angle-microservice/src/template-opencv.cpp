#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height"))) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    } else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Mutexes in order to protect the integrity of the threads

            // GeoLocations
            opendlv::logic::sensation::Geolocation currentGeoLocation;
            std::mutex currentGeoLocationMutex;
            opendlv::logic::sensation::Geolocation previousGeoLocation;
            std::mutex previousGeoLocationMutex;
            bool hasPreviousHeading = false;

            // Current speed
            opendlv::proxy::GroundSpeedReading currentSpeed;
            std::mutex currentSpeedMutex;

            // Original groundSteeringRequest
            opendlv::proxy::GroundSteeringRequest currentGSR;
            std::mutex currentGSRMutex;

            // Performance tracking variables
            int successCount = 0;
            int frameCount = 0;
            float successRate = 0.0f;

            // Lambda to get the GeoLocaition
            auto onGeoLocation = [&currentGeoLocation, &currentGeoLocationMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(currentGeoLocationMutex);
                currentGeoLocation = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received GeoLocation: Heading=" << currentGeoLocation.heading() << std::endl;
                }
            };
            od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(), onGeoLocation);

            // Lambda to get the speed
            auto onSpeed = [&currentSpeed, &currentSpeedMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(currentSpeedMutex);
                currentSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received Speed: " << currentSpeed.groundSpeed() << std::endl;
                }
            };
            od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), onSpeed);

            // Lambda to get the ground steering request
            auto onGroundSteeringRequest = [&currentGSR, &currentGSRMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(currentGSRMutex);
                currentGSR = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received GroundSteeringRequest: " << currentGSR.groundSteering() << std::endl;
                }
            };
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            while (od4.isRunning()) {
                cv::Mat img;
                sharedMemory->wait();
                sharedMemory->lock();
                {
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                cv::Rect topHalf(0, 0, WIDTH, HEIGHT / 2);
                img(topHalf) = cv::Scalar(0, 0, 0, 0);
                cv::Rect bottomPart(0, 370, WIDTH, HEIGHT - 370);
                img(bottomPart) = cv::Scalar(0, 0, 0, 0);
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0, 0, 255));

                float predictedSteering = 0.0f;
                float deltaHeading = 0.0f;
                float currentVelocity = 0.0f;
                float originalSteering = 0.0f;
                bool hasCurrentHeading = false;
                bool hasSpeed = false;
                bool hasOriginalSteering = false;

                {
                    std::lock_guard<std::mutex> lck(currentGeoLocationMutex);
                    hasCurrentHeading = true;
                }

                {
                    std::lock_guard<std::mutex> lck(currentSpeedMutex);
                    currentVelocity = currentSpeed.groundSpeed(); // Groundspeed is received, using the opendlv standard message

                    // DISCLAIMER; THE VELOCITY IS CURRENTLY **ALWAYS** 0
                    if (currentVelocity < 0.01) {
                        currentVelocity = 1.0;
                        std::cout << "Using default speed: " << currentVelocity << std::endl;
                    }

                    hasSpeed = true;
                }

                {
                    std::lock_guard<std::mutex> lck(currentGSRMutex);
                    originalSteering = currentGSR.groundSteering();
                    hasOriginalSteering = true;
                }

                if (hasCurrentHeading && hasPreviousHeading && hasSpeed && hasOriginalSteering) {
                    deltaHeading = currentGeoLocation.heading() - previousGeoLocation.heading();

                    // Calculation based on speed
                    // predictedSteering = deltaHeading * currentVelocity * 0.1f; // Example factor
                    predictedSteering = deltaHeading * 10; // trying to set predictedSteering to just the deltaheading without speed or tuning parameter

                    float difference = std::abs(predictedSteering - originalSteering);

                    if (originalSteering != 0.0f) {
                        if (difference <= 0.09f) {
                            successCount++;
                        }
                        frameCount++;
                        if (frameCount > 0) {
                            successRate = static_cast<float>(successCount) / frameCount;
                        }
                    }

                    if (VERBOSE) {
                        std::cout << "Original Steering:   " << std::fixed << std::setprecision(4) << originalSteering << std::endl;
                        std::cout << "Current Heading:    " << std::fixed << std::setprecision(4) << currentGeoLocation.heading() << std::endl;
                        std::cout << "Previous Heading:   " << std::fixed << std::setprecision(4) << previousGeoLocation.heading() << std::endl;
                        std::cout << "Delta Heading:      " << std::fixed << std::setprecision(4) << deltaHeading << std::endl;
                        std::cout << "Speed:              " << std::fixed << std::setprecision(2) << currentVelocity << std::endl;
                        std::cout << "Predicted Steering: " << std::fixed << std::setprecision(4) << predictedSteering << std::endl;
                        std::cout << "Difference (Pred - Orig): " << std::fixed << std::setprecision(4) << difference << std::endl;
                        std::cout << "Success Rate (when original != 0): " << std::fixed << std::setprecision(2) << successRate * 100.0f << "%" << std::endl;
                    } else {
                        std::cout << difference << std::endl;
                    }
                }

                // Update previousGeoLocation at the end of the loop, otherwise currentGeoLocation = previousGeoLocation
                {
                    std::lock_guard<std::mutex> lck(previousGeoLocationMutex);
                    previousGeoLocation = currentGeoLocation;
                    hasPreviousHeading = true;
                }

                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }

            if (totalSteeringCommandsConsidered > 0) {
                double successRate = static_cast<double>(successfulSteeringCommands) / totalSteeringCommandsConsidered;
                std::cout << "\n--- Program Ended ---" << std::endl;
                std::cout << "Total Steering Commands Considered: " << totalSteeringCommandsConsidered << std::endl;
                std::cout << "Successful Steering Commands: " << successfulSteeringCommands << std::endl;
                std::cout << "Total Success Rate: " << std::fixed << std::setprecision(4) << (successRate * 100.0) << "%" << std::endl;
            } else {
                std::cout << "\n--- Program Ended ---" << std::endl;
                std::cout << "No non-zero original steering commands received to calculate success rate." << std::endl;
            }
        }
        retCode = 0;
    }
    return retCode;
}
