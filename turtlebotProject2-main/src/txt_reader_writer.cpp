#include "txt_reader_writer.hpp"

TextReadWrite::TextReadWrite() {}

void TextReadWrite::ReadFile(rclcpp::Logger logger) {
    std::ifstream FileToRead(FilenameToRead);

    if (FileToRead.is_open()) {
        while (std::getline(FileToRead, FileInformation)) {
            // do something with the info
        }
        FileToRead.close();
    }
    else {
        RCLCPP_INFO(logger, "Error opening file.");
    }
}

void TextReadWrite::WriteDeliveryFile(std::string DeliveryToWrite) {
    std::ofstream DeliveryFile(DeliveryFilename);

    if (DeliveryFile.is_open()) {
        DeliveryFile << DeliveryToWrite;
        DeliveryFile.close();
    }
}

void TextReadWrite::WriteInspectionFile(std::string DamageToWrite) {
    std::ofstream InspectionFile(InspectionFilename);

    if (InspectionFile.is_open()) {
        InspectionFile << DamageToWrite
        InspectionFile.close();
    }
}