#ifndef FILE_MANIP
#define FILE_MANIP
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>


namespace FileManipulation
{
    /// @brief Check if a file exists
    /// @param path The path to the file
    /// @return True if the file exists, false otherwise
    inline static bool directoryExists(const std::string &path) 
    {
        struct stat info;
        return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
    }

    /// @brief Create a directory
    /// @param path The path to the directory
    /// @return True if the directory was created, false otherwise
    inline static bool createDirectory(const std::string &path) 
    {
        if (mkdir(path.c_str(), 0755) == 0) 
        {
            return true; // Directory created successfully
        } 
        else if (errno == EEXIST) 
        {
            return true; // Directory already exists
        } 
        else 
        {
            return false; // Failed to create directory
        }
    }

    /// @brief Create a text file
    /// @param file_path The path to the file
    /// @param content The content to write to the file
    inline static void createTextFile(const std::string &file_path, const std::string &content) 
    {
        std::ofstream file(file_path);
        if (file.is_open()) 
        {
            file << content;
            file.close();
        }
    }
}

#endif
