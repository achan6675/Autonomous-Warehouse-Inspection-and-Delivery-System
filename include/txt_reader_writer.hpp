#ifndef TXT_READER_WRITER_HPP
#define TXT_READER_WRITER_HPP

#include <string>
#include <iostream>
#include <fstream> 
#include <utility>
#include <format>

#include "rclcpp/rclcpp.hpp"

class TextReadWrite {
    public:
        TextReadWrite(); // constructor
        void WriteInspectionFile(std::string DamageToWrite); // write the inspection file
        void WriteDeliveryFile(std::string DeliveryToWrite); // write the delivery file
    private:
        std::string DeliveryFilename = "deliveries.txt"; // filename for delivery report
        std::string InspectionFilename = "damage_report.txt"; // filename for inspection report
        std::string FilenameToRead; // filename to read
        std::string FileInformation; // information read from the file

        void ReadFile(rclcpp::Logger logger); // read a file
};

#endif