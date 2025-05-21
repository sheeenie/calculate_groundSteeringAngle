/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <fstream>

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
        
        // Open CSV file for writing (will overwrite if it exists)
        //std::ofstream outFile("calculated_steering.csv");
        std::ofstream outFile("/output/calculated_steering.csv");
        outFile << "timestamp; calcSteeringAngle; orginalSteeringAngle\n";  // Header row


        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Mutexes in order to protect the integrity of the threads

            // START: GeoLocations
            opendlv::logic::sensation::Geolocation currentGeoLocation;
            std::mutex currentGeoLocationMutex;
            opendlv::logic::sensation::Geolocation previousGeoLocation;
            std::mutex previousGeoLocationMutex;
            bool hasPreviousHeading = false;
            //END: GeoLocations

            // Original groundSteeringRequest
            opendlv::proxy::GroundSteeringRequest currentGSR;
            std::mutex currentGSRMutex;

            // BEGIN: Speed Estimation Variables
            float estimatedVelocity = 0.0f;
            float lastAccelerationTimestamp = 0.0f;
            std::mutex velocityMutex;
            // END: Speed Estimation Variables

            // Performance tracking variables
            int successCount = 0;
            int frameCount = 0;
            float successRate = 0.0f;

            // variable to sacve the time
            float timestampMicroS = 0;

            // Lambda function to get the GeoLocation, executed when a geolocation is received
            auto onGeoLocation = [&currentGeoLocation, &currentGeoLocationMutex, VERBOSE](cluon::data::Envelope &&env) {
                // Lock mutex to safely update the shared memory below
                std::lock_guard<std::mutex> lck(currentGeoLocationMutex);
                // Extract geolocation from the shared memory and store it in a global variable
                currentGeoLocation = cluon::extractMessage<opendlv::logic::sensation::Geolocation>(std::move(env));
            };
            od4.dataTrigger(opendlv::logic::sensation::Geolocation::ID(), onGeoLocation); // Callback registration

            // Lambda function to get the acceleration and compute estimated speed, executed when a geolocation is received
            auto onAcceleration = [&estimatedVelocity, &lastAccelerationTimestamp, &velocityMutex, &timestampMicroS, VERBOSE](cluon::data::Envelope &&env) {
                // Extract acceleration data from the incoming message
                auto acc = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(env));
                // X-axis acceleration
                float ax = acc.accelerationX();
                // Store timestamp for calculations and printings
                timestampMicroS = env.sampleTimeStamp().microseconds();
                float timestamp = env.sampleTimeStamp().microseconds() / 1e6f;

                // Lock the mutex to safely update velocity-related variables
                std::lock_guard<std::mutex> lck(velocityMutex);
                // Only update the velocity if we have a previous timestamp to calculate time difference
                if (lastAccelerationTimestamp > 0.0f) {
                    // Calculate time difference between current and previous acceleration reading
                    float dt = timestamp - lastAccelerationTimestamp;
                    // Update velocity estimate using basic physics: v = v0 + a*t
                    estimatedVelocity += ax * dt;
                }
                lastAccelerationTimestamp = timestamp;
            };
            od4.dataTrigger(opendlv::proxy::AccelerationReading::ID(), onAcceleration); // Callback registration

            // Lambda function to get the ground steering request, executed upon receiving a ground steering request
            auto onGroundSteeringRequest = [&currentGSR, &currentGSRMutex, VERBOSE](cluon::data::Envelope &&env) {
                // Lock mutex to safely update the shared memory below
                std::lock_guard<std::mutex> lck(currentGSRMutex);
                // Extract ground steering from the shared memory and store it in a global variable
                currentGSR = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            };
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest); // Callback registration

            while (od4.isRunning()) {
                cv::Mat img;
                sharedMemory->wait();
                sharedMemory->lock();
                {
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                // Variable initialization 
                float predictedSteering = 0.0f;
                float deltaHeading = 0.0f;
                float currentVelocity = 0.0f;
                float originalSteering = 0.0f;
                bool hasCurrentHeading = false;
                bool hasOriginalSteering = false;

                // Safely sets the hasCurrentHeading and hasOriginalSteering to true, avoids race conditions.
                {
                    std::lock_guard<std::mutex> lck(currentGeoLocationMutex);
                    hasCurrentHeading = true;
                }
                {
                    std::lock_guard<std::mutex> lck(currentGSRMutex);
                    originalSteering = currentGSR.groundSteering();
                    hasOriginalSteering = true;
                }

                if (hasCurrentHeading && hasPreviousHeading && hasOriginalSteering) { // Check whether there are more readings available from the shared memory 
                    
                    // ----------------------------  Calculations in order to find the new ground steering angle ----------------------------

                    deltaHeading = currentGeoLocation.heading() - previousGeoLocation.heading();
                    // Convert the heading to radius from degrees
                    float deltaHeadingRad = deltaHeading * M_PI / 180.0f;
                    // 10Hz frame rate
                    float deltaTime = 0.1f; 
                    // Prevent div-by-zero
                    float velocity = std::max(estimatedVelocity, 0.1f); 
                    float angularVelocity = deltaHeadingRad / deltaTime;
                    // Final predicted steering, which will be compared to the original ground steering
                    predictedSteering = angularVelocity / velocity;

                    // Difference between the predicted and original ground steering
                    float difference = std::abs(predictedSteering - originalSteering);

                    if (originalSteering != 0.0f) { // Where the ground steering is not 0, check to see if the difference is +-0.09 as per the requirements
                        if (difference <= 0.09f) {
                            successCount++;
                        }
                        frameCount++;
                        if (frameCount > 0) {
                            successRate = static_cast<float>(successCount) / frameCount;
                        }
                    }

                    // Print the result of the different datapoints and calculations
                    std::cout << "group_12;" << timestampMicroS << ";" << predictedSteering << std::endl;    
                    
                }

                // gets a pop-up window with more information displayed 
                if (VERBOSE) {
                    // Format variables
                    int lineSpacing = 30;
                    int baseY = 30;

                    // Timestamp (already present, included for completeness)
                    cv::putText(img, "Timestamp: " + std::to_string(static_cast<int>(timestampMicroS)), 
                                cv::Point(10, baseY), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                    // Predicted Steering
                    cv::putText(img, "Predicted Steering: " + std::to_string(predictedSteering), 
                                cv::Point(10, baseY + lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

                    // Original Steering
                    cv::putText(img, "Original Steering: " + std::to_string(originalSteering), 
                                cv::Point(10, baseY + 2 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

                    // Current Heading
                    cv::putText(img, "Current Heading: " + std::to_string(currentGeoLocation.heading()), 
                                cv::Point(10, baseY + 4 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                    // Previous Heading
                    cv::putText(img, "Previous Heading: " + std::to_string(previousGeoLocation.heading()), 
                                cv::Point(10, baseY + 5 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                    // Delta Heading
                    cv::putText(img, "Delta Heading: " + std::to_string(deltaHeading), 
                                cv::Point(10, baseY + 6 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

                    // Difference (Pred - Orig)
                    cv::putText(img, "Difference (Pred - Orig): " + std::to_string(std::abs(predictedSteering - originalSteering)), 
                                cv::Point(10, baseY + 7 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

                    // Success Rate
                    cv::putText(img, "Success Rate: " + std::to_string(successRate * 100.0f) + "%", 
                                cv::Point(10, baseY + 8 * lineSpacing), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

                    // Show the image
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);

                }

                // The previous geo location has to be updated in the end, in order to avoid it being overwritten with the current geolocation
                {
                    std::lock_guard<std::mutex> lck(previousGeoLocationMutex);
                    previousGeoLocation = currentGeoLocation;
                    hasPreviousHeading = true;
                }
            }
        }
        outFile.close(); // Used for writing to a csv file
        retCode = 0;
    }
    return retCode;
}