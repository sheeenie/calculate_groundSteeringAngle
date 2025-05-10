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

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications 
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <mutex>  // Added for mutex

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Variables and mutex for the ground steering angle
            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;

            // Variables and mutexes for acceleration and angular velocity
            float accelerationX = 0.0f;
            std::mutex accelMutex;
            float angularVelocityZ = 0.0f;
            std::mutex angularMutex;

            // Variables to calculate the time difference 
            uint64_t lastMicroseconds = 0; // To store previous timestamp
            float velocity = 0.0f;
            float steeringAngle = 0.0f;


            // Lambda for ground steering
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            // Lambda for acceleration x-value
            auto onAccelerationReading = [&accelerationX, &accelMutex](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(accelMutex);
                auto acc = cluon::extractMessage<opendlv::proxy::AccelerationReading>(std::move(env));
                accelerationX = acc.accelerationX();
                std::cout << "lambda: accelerationX = " << accelerationX << " m/s^2" << std::endl;
                std::cout << "timestamp = " << env.sampleTimeStamp().seconds() << " seconds" << std::endl;
            };
            
            // Lambda for angular velocity reading z-value 
            auto onAngularVelocityReading = [&angularVelocityZ, &angularMutex](cluon::data::Envelope &&env){
                std::lock_guard<std::mutex> lck(angularMutex);
                auto angVel = cluon::extractMessage<opendlv::proxy::AngularVelocityReading>(std::move(env));
                angularVelocityZ = angVel.angularVelocityZ();
                std::cout << "lambda: angularVelocityZ = " << angularVelocityZ << " rad/s" << std::endl;
                std::cout << "timestamp = " << env.sampleTimeStamp().seconds() << " seconds" << std::endl;
            };
            

            // Register data triggers
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
            od4.dataTrigger(opendlv::proxy::AccelerationReading::ID(), onAccelerationReading); // NEW
            od4.dataTrigger(opendlv::proxy::AngularVelocityReading::ID(), onAngularVelocityReading); // NEW

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
                // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }

                // Get the sample time point when the current frame was captured
                cluon::data::TimeStamp sampleTimePoint = sharedMemory->getTimeStamp().second;
                uint64_t microseconds = cluon::time::toMicroseconds(sampleTimePoint);

                // Calculate delta time in seconds
                float deltaTime = (lastMicroseconds > 0) ? (microseconds - lastMicroseconds) / 1e6f : 0.0f;

                // Update last timestamp
                lastMicroseconds = microseconds;

                // Calculate velocity and steering angle
                {
                    std::lock_guard<std::mutex> lck(accelMutex);
                    velocity = accelerationX * deltaTime; // simple integration
                }

                {
                    std::lock_guard<std::mutex> lck(angularMutex);
                    if (velocity != 0.0f) {
                        steeringAngle = angularVelocityZ / velocity;
                    } else {
                        steeringAngle = 0.0f; // avoid division by zero
                    }
                }

                // Print the calculated values
                std::cout << "main: deltaTime = " << deltaTime << " s" << std::endl;
                std::cout << "main: velocity = " << velocity << " m/s" << std::endl;
                std::cout << "main: steeringAngle = " << steeringAngle << " rad" << std::endl;

                sharedMemory->unlock();

                // Get current UTC time
                cluon::data::TimeStamp now = cluon::time::now();
                std::time_t time_now = static_cast<std::time_t>(now.seconds());
                std::tm* utcTime = std::gmtime(&time_now);

                char utcTimeBuffer[30];
                std::strftime(utcTimeBuffer, sizeof(utcTimeBuffer), "%Y-%m-%dT%H:%M:%SZ", utcTime);

                // Convert the sample time point to a readable format
                std::string sampleTimeStr = std::to_string(microseconds);

                // Add text overlays to the image
                cv::putText(img, "Now: " +std::string(utcTimeBuffer) + "; ----- ts: " + sampleTimeStr + " ----- ", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                // Show deltaTime
                cv::putText(img, "deltaTime: " + std::to_string(deltaTime) + " s", 
                cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 1);

                // Show velocity
                cv::putText(img, "velocity: " + std::to_string(velocity) + " m/s", 
                cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 1);

                // Show steeringAngle
                cv::putText(img, "steeringAngle: " + std::to_string(steeringAngle) + " rad", 
                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cv::Scalar(255, 255, 255), 1);

                

                // Draw a red rectangle
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                // Access the latest received ground steering
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
                }

                // NEW: Access latest acceleration
                {
                    std::lock_guard<std::mutex> lck(accelMutex);
                    std::cout << "main: accelerationX = " << accelerationX << " m/s^2" << std::endl;
                }

                // NEW: Access latest angular velocity
                {
                    std::lock_guard<std::mutex> lck(angularMutex);
                    std::cout << "main: angularVelocityZ = " << angularVelocityZ << " rad/s" << std::endl;
                }

                
                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}
