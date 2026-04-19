// MTRX3760 2025 Project 2: Warehouse Robot DevKit
// File: text_file_generator.hpp
// Author(s): Amelia Chan and Imogen Coward
//
// Generates and manages text files for deliveries, damage reports, and inspections.

#ifndef TEXT_FILE_GENERATOR_HPP_
#define TEXT_FILE_GENERATOR_HPP_

#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "txt_reader_writer.hpp"
#include <memory>
#include <vector>

/**
 * @brief Structure to store delivery or damage entries.
 */
struct ItemEntry
{
    int id;     ///< Unique ID for the item.
    int row;    ///< Row location of the item.
    int column; ///< Column location of the item.
};

/**
 * @brief Class for generating and managing text files related to warehouse operations.
 */
class TextFileGenerator {
public:
    /**
     * @brief Default constructor.
     */
    TextFileGenerator(){};

    /**
     * @brief Default destructor.
     */
    ~TextFileGenerator(){};

    /**
     * @brief Generates a delivery file from a vector of item entries.
     * @param entries Vector of ItemEntry to include in the delivery file.
     */
    void GetDeliveryFile(const std::vector<ItemEntry> &entries);

    /**
     * @brief Generates a damage/inspection file from a vector of item entries.
     * @param entries Vector of ItemEntry to include in the damage report.
     */
    void GetDamageFile(const std::vector<ItemEntry> &entries);

    /**
     * @brief Writes the inspection file with given content.
     * @param DamageToWrite String content to write to the inspection file.
     */
    void WriteInspectionFile(const std::string &DamageToWrite);

    /**
     * @brief Writes the delivery file with given content.
     * @param DeliveryToWrite String content to write to the delivery file.
     */
    void WriteDeliveryFile(const std::string &DeliveryToWrite);

private:
    std::string DeliveryFilename = "deliveries.txt";   ///< Filename for delivery report.
    std::string InspectionFilename = "damage_report.txt"; ///< Filename for inspection report.
    std::string FilenameToRead;                        ///< Name of the file to read.
    std::string FileInformation;                       ///< Information read from the file.

    /**
     * @brief Reads the contents of a file into FileInformation.
     * @param logger Logger to output read errors or info.
     */
    void ReadFile(rclcpp::Logger logger);
};

#endif
