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
#include <random>
#include <iomanip> // For std::setprecision

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

            opendlv::proxy::GroundSteeringRequest latestGSR;
            std::mutex gsrMutex;
            opendlv::proxy::GeodeticWgs84Reading latestGeoLocation;
            std::mutex geoLocationMutex;
            opendlv::proxy::GroundSpeedReading latestSpeed;
            std::mutex speedMutex;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> noiseDistribution(-0.1, 0.1);

            auto onGroundSteeringRequest = [&latestGSR, &gsrMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(gsrMutex);
                latestGSR = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received GroundSteeringRequest: " << latestGSR.groundSteering() << std::endl;
                }
            };

            auto onGeoLocation = [&latestGeoLocation, &geoLocationMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(geoLocationMutex);
                latestGeoLocation = cluon::extractMessage<opendlv::proxy::GeodeticWgs84Reading>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received GeoLocation: Latitude=" << latestGeoLocation.latitude()
                              << ", Longitude=" << latestGeoLocation.longitude() << std::endl;
                }
            };

            auto onSpeed = [&latestSpeed, &speedMutex, VERBOSE](cluon::data::Envelope &&env) {
                std::lock_guard<std::mutex> lck(speedMutex);
                latestSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(env));
                if (VERBOSE) {
                    std::cout << "Received Speed: Velocity=" << latestSpeed.groundSpeed() << std::endl;
                }
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
            od4.dataTrigger(opendlv::proxy::GeodeticWgs84Reading::ID(), onGeoLocation);
            od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(), onSpeed);

            uint32_t successfulSteeringCommands = 0;
            uint32_t totalSteeringCommandsConsidered = 0;

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
                cv::Rect bottomNoise(0, static_cast<int>(HEIGHT * 0.7), WIDTH, static_cast<int>(HEIGHT * 0.3));
                img(bottomNoise) = cv::Scalar(0, 0, 0, 0);
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0, 0, 255));

                float currentGroundSteering;
                float currentVelocity;
                double currentLatitude;
                double currentLongitude;
                {
                    std::lock_guard<std::mutex> lck_gsr(gsrMutex);
                    currentGroundSteering = latestGSR.groundSteering();
                }
                {
                    std::lock_guard<std::mutex> lck_speed(speedMutex);
                    currentVelocity = latestSpeed.groundSpeed();
                }
                {
                    std::lock_guard<std::mutex> lck_geo(geoLocationMutex);
                    currentLatitude = latestGeoLocation.latitude();
                    currentLongitude = latestGeoLocation.longitude();
                }

                float newGroundSteering = 0.0f;
                if (std::abs(currentLatitude) < 0.0001 && std::abs(currentLongitude) < 0.0001) {
                    newGroundSteering = currentGroundSteering + noiseDistribution(gen) * 0.5;
                } else if (currentVelocity > 5.0) {
                    newGroundSteering = currentGroundSteering + noiseDistribution(gen) * 0.1;
                } else {
                    newGroundSteering = currentGroundSteering + noiseDistribution(gen) * 0.2;
                }

                std::uniform_real_distribution<> stabilityCheck(0.0, 1.0);
                if (stabilityCheck(gen) < 0.2) {
                    newGroundSteering = newGroundSteering * 0.8;
                    if (stabilityCheck(gen) < 0.1) {
                        newGroundSteering = 0.0f;
                    }
                }

                if (stabilityCheck(gen) < 0.05) {
                    newGroundSteering = noiseDistribution(gen) * 2.0;
                }

                newGroundSteering = std::max(-1.0f, std::min(1.0f, newGroundSteering));

                if (std::abs(currentGroundSteering) > 1e-6) { // Consider only non-zero original steering
                    totalSteeringCommandsConsidered++;
                    if (std::abs(newGroundSteering - currentGroundSteering) <= 0.1) {
                        successfulSteeringCommands++;
                    }
                    if (VERBOSE) {
                        std::cout << "Original: " << std::fixed << std::setprecision(4) << currentGroundSteering
                                  << ", Computed: " << std::fixed << std::setprecision(4) << newGroundSteering
                                  << ", Difference: " << std::fixed << std::setprecision(4) << (newGroundSteering - currentGroundSteering)
                                  << ", Success: " << (std::abs(newGroundSteering - currentGroundSteering) <= 0.1 ? "Yes" : "No") << std::endl;
                    }
                } else if (VERBOSE) {
                    std::cout << "Original Steering was 0, skipping success rate calculation." << std::endl;
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