#include "text_file_generator.hpp"
#include <fstream>



void TextFileGenerator::GetDeliveryFile(const std::vector<ItemEntry> &entries)
{
    for (const auto &entry : entries) {
        char string_buffer[100];
        snprintf(string_buffer, sizeof(string_buffer), "DELIVERY → ID: %d X: %d Y: %d", entry.id, entry.row, entry.column);
        std::string DeliveryToWrite = string_buffer;
        WriteDeliveryFile(DeliveryToWrite); 
    }
}

void TextFileGenerator::GetDamageFile(const std::vector<ItemEntry> &entries)
{
        std::cout << " opening file" << std::endl;

    for (const auto &entry : entries) {
        char string_buffer[100];
        snprintf(string_buffer, sizeof(string_buffer), "DAMAGE → ID: %d X: %d Y: %d", entry.id, entry.row, entry.column);
        std::string DamageToWrite = string_buffer;
        WriteInspectionFile(DamageToWrite);
    }
}


void TextFileGenerator::ReadFile(rclcpp::Logger logger) {
    std::ifstream FileToRead(FilenameToRead);

    if (FileToRead.is_open()) {
        while (std::getline(FileToRead, FileInformation)) {
            // do something with the info
        }
        FileToRead.close();
    }
    else {
        std::cout << "error opening file" << std::endl;
    }
}
void TextFileGenerator::WriteDeliveryFile(const std::string &DeliveryToWrite) {
    std::ofstream DeliveryFile(DeliveryFilename, std::ios::app);  // append mode
    if (DeliveryFile.is_open()) {
        DeliveryFile << DeliveryToWrite << std::endl;
        DeliveryFile.close();
    }
}

void TextFileGenerator::WriteInspectionFile(const std::string &DamageToWrite) {
    std::ofstream InspectionFile(InspectionFilename, std::ios::app);  // append mode
    if (InspectionFile.is_open()) {
        InspectionFile << DamageToWrite << std::endl; 
        InspectionFile.close();
    }
}
