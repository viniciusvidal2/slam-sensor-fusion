#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>

bool directoryExists(const std::string &path) 
{
    struct stat info;
    return (stat(path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR));
}

bool createDirectory(const std::string &path) 
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

void createTextFile(const std::string &filePath, const std::string &content) 
{
    std::ofstream file(filePath);
    if (file.is_open()) 
    {
        file << content;
        file.close();
    }
}
