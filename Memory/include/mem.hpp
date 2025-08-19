/**
 * @file mem.hpp
 * @brief Windows Process Memory Management Library
 *
 * This header provides a comprehensive C++ wrapper for Windows process memory operations.
 * It includes functionality for process attachment, memory reading/writing, pattern scanning,
 * and pointer chain traversal. The library is designed for educational purposes and legitimate
 * debugging/reverse engineering tasks.
 *
 * @note This library requires Windows API and appropriate process privileges.
 * @note All memory operations may throw std::runtime_error on failure.
 *
 * @author edohb
 * @version 1.0
 */

#pragma once

#include <windows.h>
#include <tlhelp32.h>
#include <psapi.h>
#include <string>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <algorithm>

/**
 * @brief A 3D vector structure for representing spatial coordinates
 *
 * This structure represents a three-dimensional vector with floating-point components.
 * Commonly used for position, velocity, or direction vectors in 3D applications.
 */
struct Vector3 {
    float x; ///< X-component of the vector
    float y; ///< Y-component of the vector
    float z; ///< Z-component of the vector

    /**
     * @brief Default constructor initializing all components to zero
     */
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}

    /**
     * @brief Parameterized constructor
     * @param x X-component value
     * @param y Y-component value
     * @param z Z-component value
     */
    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

/**
 * @brief Windows Process Memory Management Class
 *
 * The Mem class provides a comprehensive interface for interacting with Windows process memory.
 * It supports process attachment, memory reading/writing operations, pointer chain traversal,
 * pattern scanning, and various data type operations.
 *
 * @note This class requires appropriate process privileges to access target process memory.
 * @note All operations may throw std::runtime_error on failure.
 *
 * @example
 * ```cpp
 * try {
 *     Mem mem("notepad.exe");
 *     uintptr_t baseAddr = mem.GetModuleBaseAddress("notepad.exe");
 *     int value = mem.ReadInt(baseAddr + 0x1000);
 *     mem.WriteInt(baseAddr + 0x1000, 42);
 * } catch (const std::runtime_error& e) {
 *     std::cerr << "Error: " << e.what() << std::endl;
 * }
 * ```
 */
class Mem {
private:
    HANDLE processHandle;   ///< Handle to the target process
    DWORD processId;        ///< Process ID of the target process
    std::string processName; ///< Name of the target process

    /**
     * @brief Finds the process ID by process name
     * @param processName Name of the process to find
     * @return Process ID if found, 0 otherwise
     */
    DWORD FindProcessId(const std::string& processName);

    /**
     * @brief Template function to convert byte array to typed value
     * @tparam T The type to convert to
     * @param bytes Byte array containing the data
     * @param offset Offset within the byte array (default: 0)
     * @return Value of type T extracted from the byte array
     * @throws std::runtime_error if not enough bytes available
     */
    template<typename T>
    T BytesToValue(const std::vector<uint8_t>& bytes, size_t offset = 0);

public:
    /**
     * @brief Constructor that attaches to a process by name
     * @param procName Name of the process to attach to (e.g., "notepad.exe")
     * @throws std::runtime_error if the process cannot be found or attached
     *
     * @example
     * ```cpp
     * Mem mem("notepad.exe"); // Attaches to notepad.exe
     * ```
     */
    Mem(const std::string& procName);

    /**
     * @brief Destructor that properly closes the process handle
     */
    ~Mem();

    /**
     * @brief Sets/changes the target process
     * @param procName Name of the process to attach to
     * @return true if successful, false otherwise
     *
     * @note This method can be used to switch between different processes
     */
    bool SetProcess(const std::string& procName);

    /**
     * @brief Gets the process handle
     * @return Handle to the target process
     */
    HANDLE GetProcessHandle() const { return processHandle; }

    /**
     * @brief Gets the process ID
     * @return Process ID of the target process
     */
    DWORD GetProcessId() const { return processId; }

    /**
     * @brief Gets the process name
     * @return Name of the target process
     */
    const std::string& GetProcessName() const { return processName; }

    /**
     * @brief Gets the base address of a module within the target process
     * @param moduleName Name of the module (e.g., "kernel32.dll", "notepad.exe")
     * @return Base address of the module, or 0 if not found
     * @throws std::runtime_error if module name is empty or process is invalid
     *
     * @note For the main executable, you can use the process name or it will be found as the first module
     */
    uintptr_t GetModuleBaseAddress(const std::string& moduleName);

    /**
     * @brief Reads a pointer value from memory
     * @param address Memory address to read from
     * @return Pointer value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    uintptr_t ReadPointer(uintptr_t address);

    /**
     * @brief Reads a pointer value from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Pointer value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    uintptr_t ReadPointer(uintptr_t address, int offset);

    /**
     * @brief Reads a pointer following a chain of offsets (pointer traversal)
     * @param address Starting memory address
     * @param offsets Vector of offsets to follow through the pointer chain
     * @return Final pointer value after following the complete chain
     * @throws std::runtime_error if any memory read in the chain fails
     *
     * @note This is useful for accessing nested data structures or multi-level pointers
     * @example
     * ```cpp
     * // Follow: [[baseAddress + 0x10] + 0x20] + 0x30
     * std::vector<int> offsets = {0x10, 0x20, 0x30};
     * uintptr_t finalAddr = mem.ReadPointer(baseAddress, offsets);
     * ```
     */
    uintptr_t ReadPointer(uintptr_t address, const std::vector<int>& offsets);

    /**
     * @brief Convenience overload for 2-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2);

    /**
     * @brief Convenience overload for 3-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @param offset3 Third offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, int offset3);

    /**
     * @brief Convenience overload for 4-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @param offset3 Third offset
     * @param offset4 Fourth offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4);

    /**
     * @brief Convenience overload for 5-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @param offset3 Third offset
     * @param offset4 Fourth offset
     * @param offset5 Fifth offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5);

    /**
     * @brief Convenience overload for 6-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @param offset3 Third offset
     * @param offset4 Fourth offset
     * @param offset5 Fifth offset
     * @param offset6 Sixth offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5, int offset6);

    /**
     * @brief Convenience overload for 7-level pointer chain
     * @param address Starting address
     * @param offset1 First offset
     * @param offset2 Second offset
     * @param offset3 Third offset
     * @param offset4 Fourth offset
     * @param offset5 Fifth offset
     * @param offset6 Sixth offset
     * @param offset7 Seventh offset
     * @return Final pointer value
     */
    uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5, int offset6, int offset7);

    /**
     * @brief Reads raw bytes from memory
     * @param address Memory address to read from
     * @param size Number of bytes to read
     * @return Vector containing the read bytes
     * @throws std::runtime_error if memory read fails
     *
     * @note The returned vector size may be smaller than requested if fewer bytes were read
     */
    std::vector<uint8_t> ReadBytes(uintptr_t address, size_t size);

    /**
     * @brief Reads raw bytes from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param size Number of bytes to read
     * @return Vector containing the read bytes
     * @throws std::runtime_error if memory read fails
     */
    std::vector<uint8_t> ReadBytes(uintptr_t address, int offset, size_t size);

    /**
     * @brief Reads a 32-bit signed integer from memory
     * @param address Memory address to read from
     * @return Integer value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    int ReadInt(uintptr_t address);

    /**
     * @brief Reads a 32-bit signed integer from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Integer value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    int ReadInt(uintptr_t address, int offset);

    /**
     * @brief Reads a 32-bit floating-point value from memory
     * @param address Memory address to read from
     * @return Float value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    float ReadFloat(uintptr_t address);

    /**
     * @brief Reads a 32-bit floating-point value from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Float value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    float ReadFloat(uintptr_t address, int offset);

    /**
     * @brief Reads a Vector3 structure from memory
     * @param address Memory address to read from
     * @return Vector3 value containing x, y, z components
     * @throws std::runtime_error if memory read fails
     *
     * @note Assumes the Vector3 is stored as three consecutive 32-bit floats
     */
    Vector3 ReadVector3(uintptr_t address);

    /**
     * @brief Reads a Vector3 structure from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Vector3 value containing x, y, z components
     * @throws std::runtime_error if memory read fails
     */
    Vector3 ReadVector3(uintptr_t address, int offset);

    /**
     * @brief Reads a 16-bit signed integer from memory
     * @param address Memory address to read from
     * @return Short value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    short ReadShort(uintptr_t address);

    /**
     * @brief Reads a 16-bit signed integer from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Short value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    short ReadShort(uintptr_t address, int offset);

    /**
     * @brief Reads a 16-bit unsigned integer from memory
     * @param address Memory address to read from
     * @return Unsigned short value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    unsigned short ReadUShort(uintptr_t address);

    /**
     * @brief Reads a 16-bit unsigned integer from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Unsigned short value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    unsigned short ReadUShort(uintptr_t address, int offset);

    /**
     * @brief Reads a 32-bit unsigned integer from memory
     * @param address Memory address to read from
     * @return Unsigned integer value at the specified address
     * @throws std::runtime_error if memory read fails
     */
    unsigned int ReadUInt(uintptr_t address);

    /**
     * @brief Reads a 32-bit unsigned integer from memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @return Unsigned integer value at the specified address + offset
     * @throws std::runtime_error if memory read fails
     */
    unsigned int ReadUInt(uintptr_t address, int offset);

    /**
     * @brief Reads a 4x4 matrix from memory
     * @param address Memory address to read from
     * @return Vector containing 16 float values representing the 4x4 matrix
     * @throws std::runtime_error if memory read fails
     *
     * @note The matrix is read as 16 consecutive 32-bit floats
     * @note Matrix layout depends on the target application (row-major vs column-major)
     */
    std::vector<float> ReadMatrix4x4(uintptr_t address);

    /**
     * @brief Writes raw bytes to memory
     * @param address Memory address to write to
     * @param data Vector containing bytes to write
     * @return true if write was successful, false otherwise
     *
     * @note Requires appropriate write permissions to the target process
     */
    bool WriteBytes(uintptr_t address, const std::vector<uint8_t>& data);

    /**
     * @brief Writes raw bytes to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param data Vector containing bytes to write
     * @return true if write was successful, false otherwise
     */
    bool WriteBytes(uintptr_t address, int offset, const std::vector<uint8_t>& data);

    /**
     * @brief Writes a 32-bit signed integer to memory
     * @param address Memory address to write to
     * @param value Integer value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteInt(uintptr_t address, int value);

    /**
     * @brief Writes a 32-bit signed integer to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param value Integer value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteInt(uintptr_t address, int offset, int value);

    /**
     * @brief Writes a 16-bit signed integer to memory
     * @param address Memory address to write to
     * @param value Short value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteShort(uintptr_t address, short value);

    /**
     * @brief Writes a 16-bit signed integer to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param value Short value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteShort(uintptr_t address, int offset, short value);

    /**
     * @brief Writes a 16-bit unsigned integer to memory
     * @param address Memory address to write to
     * @param value Unsigned short value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteUShort(uintptr_t address, unsigned short value);

    /**
     * @brief Writes a 16-bit unsigned integer to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param value Unsigned short value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteUShort(uintptr_t address, int offset, unsigned short value);

    /**
     * @brief Writes a 32-bit unsigned integer to memory
     * @param address Memory address to write to
     * @param value Unsigned integer value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteUInt(uintptr_t address, unsigned int value);

    /**
     * @brief Writes a 32-bit unsigned integer to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param value Unsigned integer value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteUInt(uintptr_t address, int offset, unsigned int value);

    /**
     * @brief Writes a Vector3 structure to memory
     * @param address Memory address to write to
     * @param value Vector3 value to write
     * @return true if write was successful, false otherwise
     *
     * @note Writes the Vector3 as three consecutive 32-bit floats (x, y, z)
     */
    bool WriteVector3(uintptr_t address, const Vector3& value);

    /**
     * @brief Writes a Vector3 structure to memory with offset
     * @param address Base memory address
     * @param offset Offset to add to the address
     * @param value Vector3 value to write
     * @return true if write was successful, false otherwise
     */
    bool WriteVector3(uintptr_t address, int offset, const Vector3& value);

    /**
     * @brief Writes NOP instructions (0x90) to memory
     * @param address Memory address to write to
     * @param length Number of NOP bytes to write
     * @return true if write was successful, false otherwise
     *
     * @note This is commonly used for code patching to disable instructions
     * @note Each NOP instruction is a single byte with value 0x90
     */
    bool WriteNop(uintptr_t address, size_t length);

    /**
     * @brief Scans for a byte pattern within a module's memory
     * @param moduleName Name of the module to search in
     * @param pattern Hexadecimal pattern string (e.g., "48 8B 05 ? ? ? ?")
     * @return Address where pattern was found, or 0 if not found
     * @throws std::runtime_error if module not found or pattern invalid
     *
     * @note Pattern should be space-separated hex bytes
     * @note Wildcards (? or ??) are not supported in this implementation
     *
     * @example
     * ```cpp
     * uintptr_t addr = mem.ScanPattern("game.exe", "48 8B 05 12 34 56 78");
     * ```
     */
    uintptr_t ScanPattern(const std::string& moduleName, const std::string& pattern);

    /**
     * @brief Scans for a byte pattern within a byte array
     * @param haystack Byte array to search in
     * @param needle Byte pattern to search for
     * @return Index where pattern was found, or -1 if not found
     *
     * @note This is a helper function used internally by the string pattern scanner
     */
    int ScanPattern(const std::vector<uint8_t>& haystack, const std::vector<uint8_t>& needle);
};

/**
 * @brief Constructor implementation
 *
 * Initializes the Mem object and attempts to attach to the specified process.
 * If the process cannot be found or attached, throws an exception.
 */
Mem::Mem(const std::string& procName)
    : processHandle(nullptr), processId(0), processName(procName) {
    if (!SetProcess(procName)) {
        throw std::runtime_error("Process was not found");
    }
}

/**
 * @brief Destructor implementation
 *
 * Properly closes the process handle to prevent resource leaks.
 */
Mem::~Mem() {
    if (processHandle && processHandle != INVALID_HANDLE_VALUE) {
        CloseHandle(processHandle);
    }
}

/**
 * @brief Finds a process ID by process name using Windows Toolhelp API
 *
 * Creates a snapshot of all running processes and searches for the specified
 * process name, converting from wide characters to UTF-8 for comparison.
 *
 * @param processName The name of the process to find
 * @return The process ID if found, 0 otherwise
 */
DWORD Mem::FindProcessId(const std::string& processName) {
    PROCESSENTRY32W processEntry;
    processEntry.dwSize = sizeof(PROCESSENTRY32W);

    HANDLE processSnapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (processSnapshot == INVALID_HANDLE_VALUE) {
        return 0;
    }

    if (Process32FirstW(processSnapshot, &processEntry)) {
        do {
            int size = WideCharToMultiByte(CP_UTF8, 0, processEntry.szExeFile, -1, nullptr, 0, nullptr, nullptr);
            std::string narrowProcessName(size - 1, '\0');
            WideCharToMultiByte(CP_UTF8, 0, processEntry.szExeFile, -1, &narrowProcessName[0], size, nullptr, nullptr);

            if (processName == narrowProcessName) {
                CloseHandle(processSnapshot);
                return processEntry.th32ProcessID;
            }
        } while (Process32NextW(processSnapshot, &processEntry));
    }

    CloseHandle(processSnapshot);
    return 0;
}

/**
 * @brief Sets the target process for memory operations
 *
 * Finds the process by name and opens a handle with full access rights.
 * This handle is used for all subsequent memory operations.
 *
 * @param procName Name of the process to attach to
 * @return true if successful, false if process not found or access denied
 */
bool Mem::SetProcess(const std::string& procName) {
    processId = FindProcessId(procName);
    if (processId == 0) {
        return false;
    }

    processHandle = OpenProcess(PROCESS_ALL_ACCESS, FALSE, processId);
    if (processHandle == nullptr || processHandle == INVALID_HANDLE_VALUE) {
        return false;
    }

    processName = procName;
    return true;
}

/**
 * @brief Gets the base address of a module within the target process
 *
 * Enumerates all modules in the target process and searches for the specified
 * module by name. Handles both DLL names and executable names.
 *
 * @param moduleName Name of the module to find
 * @return Base address of the module, or 0 if not found
 * @throws std::runtime_error if module name is empty or process handle is invalid
 */
uintptr_t Mem::GetModuleBaseAddress(const std::string& moduleName) {
    if (moduleName.empty()) {
        throw std::runtime_error("moduleName was either null or empty.");
    }

    if (processHandle == nullptr) {
        throw std::runtime_error("process is invalid, check your init.");
    }

    HMODULE modules[1024];
    DWORD bytesNeeded;

    if (EnumProcessModules(processHandle, modules, sizeof(modules), &bytesNeeded)) {
        for (unsigned int i = 0; i < (bytesNeeded / sizeof(HMODULE)); i++) {
            char modulePath[MAX_PATH];
            if (GetModuleFileNameExA(processHandle, modules[i], modulePath, sizeof(modulePath))) {
                std::string moduleFile = modulePath;
                size_t pos = moduleFile.find_last_of("\\/");
                if (pos != std::string::npos) {
                    moduleFile = moduleFile.substr(pos + 1);
                }

                if (moduleFile == moduleName || (moduleName.find(".exe") != std::string::npos && i == 0)) {
                    return reinterpret_cast<uintptr_t>(modules[i]);
                }
            }
        }
    }

    return 0;
}

/**
 * @brief Template function to convert byte array to typed value
 *
 * Safely converts a portion of a byte array to the specified type T.
 * Includes bounds checking to prevent buffer overruns.
 *
 * @tparam T The type to convert to
 * @param bytes Byte array containing the data
 * @param offset Offset within the byte array
 * @return Value of type T extracted from the byte array
 * @throws std::runtime_error if not enough bytes available for conversion
 */
template<typename T>
T Mem::BytesToValue(const std::vector<uint8_t>& bytes, size_t offset) {
    if (offset + sizeof(T) > bytes.size()) {
        throw std::runtime_error("Not enough bytes to convert to requested type");
    }
    return *reinterpret_cast<const T*>(&bytes[offset]);
}

/**
 * @brief Reads a pointer value from memory
 *
 * @param address Memory address to read from
 * @return Pointer value at the specified address
 */
uintptr_t Mem::ReadPointer(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(uintptr_t));
    return BytesToValue<uintptr_t>(buffer);
}

/**
 * @brief Reads a pointer value from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Pointer value at the specified address + offset
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset) {
    auto buffer = ReadBytes(address + offset, sizeof(uintptr_t));
    return BytesToValue<uintptr_t>(buffer);
}

/**
 * @brief Reads a pointer following a chain of offsets
 *
 * Traverses through a pointer chain by following each offset in sequence.
 * Each step dereferences the current address plus offset to get the next address.
 *
 * @param address Starting memory address
 * @param offsets Vector of offsets to follow through the pointer chain
 * @return Final pointer value after following the complete chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, const std::vector<int>& offsets) {
    uintptr_t currentAddress = address;
    for (int offset : offsets) {
        currentAddress = ReadPointer(currentAddress + offset);
    }
    return currentAddress;
}

/**
 * @brief Convenience overload for 2-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2) {
    return ReadPointer(address, std::vector<int>{offset1, offset2});
}

/**
 * @brief Convenience overload for 3-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2, int offset3) {
    return ReadPointer(address, std::vector<int>{offset1, offset2, offset3});
}

/**
 * @brief Convenience overload for 4-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4) {
    return ReadPointer(address, std::vector<int>{offset1, offset2, offset3, offset4});
}

/**
 * @brief Convenience overload for 5-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5) {
    return ReadPointer(address, std::vector<int>{offset1, offset2, offset3, offset4, offset5});
}

/**
 * @brief Convenience overload for 6-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5, int offset6) {
    return ReadPointer(address, std::vector<int>{offset1, offset2, offset3, offset4, offset5, offset6});
}

/**
 * @brief Convenience overload for 7-level pointer chain
 */
uintptr_t Mem::ReadPointer(uintptr_t address, int offset1, int offset2, int offset3, int offset4, int offset5, int offset6, int offset7) {
    return ReadPointer(address, std::vector<int>{offset1, offset2, offset3, offset4, offset5, offset6, offset7});
}

/**
 * @brief Reads raw bytes from memory using Windows ReadProcessMemory API
 *
 * @param address Memory address to read from
 * @param size Number of bytes to read
 * @return Vector containing the read bytes
 * @throws std::runtime_error if memory read fails
 */
std::vector<uint8_t> Mem::ReadBytes(uintptr_t address, size_t size) {
    std::vector<uint8_t> buffer(size);
    SIZE_T bytesRead;

    if (!ReadProcessMemory(processHandle, reinterpret_cast<LPCVOID>(address),
        buffer.data(), size, &bytesRead)) {
        throw std::runtime_error("Failed to read process memory");
    }

    buffer.resize(bytesRead);
    return buffer;
}

/**
 * @brief Reads raw bytes from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param size Number of bytes to read
 * @return Vector containing the read bytes
 */
std::vector<uint8_t> Mem::ReadBytes(uintptr_t address, int offset, size_t size) {
    return ReadBytes(address + offset, size);
}

/**
 * @brief Reads a 32-bit signed integer from memory
 *
 * @param address Memory address to read from
 * @return Integer value at the specified address
 */
int Mem::ReadInt(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(int));
    return BytesToValue<int>(buffer);
}

/**
 * @brief Reads a 32-bit signed integer from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Integer value at the specified address + offset
 */
int Mem::ReadInt(uintptr_t address, int offset) {
    return ReadInt(address + offset);
}

/**
 * @brief Reads a 32-bit floating-point value from memory
 *
 * @param address Memory address to read from
 * @return Float value at the specified address
 */
float Mem::ReadFloat(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(float));
    return BytesToValue<float>(buffer);
}

/**
 * @brief Reads a 32-bit floating-point value from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Float value at the specified address + offset
 */
float Mem::ReadFloat(uintptr_t address, int offset) {
    return ReadFloat(address + offset);
}

/**
 * @brief Reads a Vector3 structure from memory
 *
 * Reads three consecutive 32-bit floats and constructs a Vector3 object.
 *
 * @param address Memory address to read from
 * @return Vector3 value containing x, y, z components
 */
Vector3 Mem::ReadVector3(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(float) * 3);
    return Vector3(
        BytesToValue<float>(buffer, 0),
        BytesToValue<float>(buffer, 4),
        BytesToValue<float>(buffer, 8)
    );
}

/**
 * @brief Reads a Vector3 structure from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Vector3 value containing x, y, z components
 */
Vector3 Mem::ReadVector3(uintptr_t address, int offset) {
    return ReadVector3(address + offset);
}

/**
 * @brief Reads a 16-bit signed integer from memory
 *
 * @param address Memory address to read from
 * @return Short value at the specified address
 */
short Mem::ReadShort(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(short));
    return BytesToValue<short>(buffer);
}

/**
 * @brief Reads a 16-bit signed integer from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Short value at the specified address + offset
 */
short Mem::ReadShort(uintptr_t address, int offset) {
    return ReadShort(address + offset);
}

/**
 * @brief Reads a 16-bit unsigned integer from memory
 *
 * @param address Memory address to read from
 * @return Unsigned short value at the specified address
 */
unsigned short Mem::ReadUShort(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(unsigned short));
    return BytesToValue<unsigned short>(buffer);
}

/**
 * @brief Reads a 16-bit unsigned integer from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Unsigned short value at the specified address + offset
 */
unsigned short Mem::ReadUShort(uintptr_t address, int offset) {
    return ReadUShort(address + offset);
}

/**
 * @brief Reads a 32-bit unsigned integer from memory
 *
 * @param address Memory address to read from
 * @return Unsigned integer value at the specified address
 */
unsigned int Mem::ReadUInt(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(unsigned int));
    return BytesToValue<unsigned int>(buffer);
}

/**
 * @brief Reads a 32-bit unsigned integer from memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @return Unsigned integer value at the specified address + offset
 */
unsigned int Mem::ReadUInt(uintptr_t address, int offset) {
    return ReadUInt(address + offset);
}

/**
 * @brief Reads a 4x4 matrix from memory
 *
 * Reads 16 consecutive 32-bit floats representing a 4x4 transformation matrix.
 *
 * @param address Memory address to read from
 * @return Vector containing 16 float values representing the 4x4 matrix
 */
std::vector<float> Mem::ReadMatrix4x4(uintptr_t address) {
    auto buffer = ReadBytes(address, sizeof(float) * 16);
    std::vector<float> matrix(16);

    for (int i = 0; i < 16; i++) {
        matrix[i] = BytesToValue<float>(buffer, i * sizeof(float));
    }

    return matrix;
}

/**
 * @brief Writes raw bytes to memory using Windows WriteProcessMemory API
 *
 * @param address Memory address to write to
 * @param data Vector containing bytes to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteBytes(uintptr_t address, const std::vector<uint8_t>& data) {
    SIZE_T bytesWritten;
    return WriteProcessMemory(processHandle, reinterpret_cast<LPVOID>(address),
        data.data(), data.size(), &bytesWritten);
}

/**
 * @brief Writes raw bytes to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param data Vector containing bytes to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteBytes(uintptr_t address, int offset, const std::vector<uint8_t>& data) {
    return WriteBytes(address + offset, data);
}

/**
 * @brief Writes a 32-bit signed integer to memory
 *
 * @param address Memory address to write to
 * @param value Integer value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteInt(uintptr_t address, int value) {
    std::vector<uint8_t> data(sizeof(int));
    *reinterpret_cast<int*>(data.data()) = value;
    return WriteBytes(address, data);
}

/**
 * @brief Writes a 32-bit signed integer to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param value Integer value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteInt(uintptr_t address, int offset, int value) {
    return WriteInt(address + offset, value);
}

/**
 * @brief Writes a 16-bit signed integer to memory
 *
 * @param address Memory address to write to
 * @param value Short value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteShort(uintptr_t address, short value) {
    std::vector<uint8_t> data(sizeof(short));
    *reinterpret_cast<short*>(data.data()) = value;
    return WriteBytes(address, data);
}

/**
 * @brief Writes a 16-bit signed integer to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param value Short value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteShort(uintptr_t address, int offset, short value) {
    return WriteShort(address + offset, value);
}

/**
 * @brief Writes a 16-bit unsigned integer to memory
 *
 * @param address Memory address to write to
 * @param value Unsigned short value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteUShort(uintptr_t address, unsigned short value) {
    std::vector<uint8_t> data(sizeof(unsigned short));
    *reinterpret_cast<unsigned short*>(data.data()) = value;
    return WriteBytes(address, data);
}

/**
 * @brief Writes a 16-bit unsigned integer to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param value Unsigned short value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteUShort(uintptr_t address, int offset, unsigned short value) {
    return WriteUShort(address + offset, value);
}

/**
 * @brief Writes a 32-bit unsigned integer to memory
 *
 * @param address Memory address to write to
 * @param value Unsigned integer value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteUInt(uintptr_t address, unsigned int value) {
    std::vector<uint8_t> data(sizeof(unsigned int));
    *reinterpret_cast<unsigned int*>(data.data()) = value;
    return WriteBytes(address, data);
}

/**
 * @brief Writes a 32-bit unsigned integer to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param value Unsigned integer value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteUInt(uintptr_t address, int offset, unsigned int value) {
    return WriteUInt(address + offset, value);
}

/**
 * @brief Writes a Vector3 structure to memory
 *
 * Writes the Vector3 as three consecutive 32-bit floats (x, y, z).
 *
 * @param address Memory address to write to
 * @param value Vector3 value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteVector3(uintptr_t address, const Vector3& value) {
    std::vector<uint8_t> data(sizeof(float) * 3);
    *reinterpret_cast<float*>(&data[0]) = value.x;
    *reinterpret_cast<float*>(&data[4]) = value.y;
    *reinterpret_cast<float*>(&data[8]) = value.z;
    return WriteBytes(address, data);
}

/**
 * @brief Writes a Vector3 structure to memory with offset
 *
 * @param address Base memory address
 * @param offset Offset to add to the address
 * @param value Vector3 value to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteVector3(uintptr_t address, int offset, const Vector3& value) {
    return WriteVector3(address + offset, value);
}

/**
 * @brief Writes NOP instructions (0x90) to memory
 *
 * Creates an array of NOP instructions and writes them to the specified address.
 * This is commonly used for code patching to disable instructions.
 *
 * @param address Memory address to write to
 * @param length Number of NOP bytes to write
 * @return true if write was successful, false otherwise
 */
bool Mem::WriteNop(uintptr_t address, size_t length) {
    std::vector<uint8_t> nopArray(length, 0x90);
    return WriteBytes(address, nopArray);
}

/**
 * @brief Scans for a byte pattern within a module's memory
 *
 * Searches for a hexadecimal byte pattern within the specified module.
 * The pattern string should contain space-separated hexadecimal bytes.
 *
 * @param moduleName Name of the module to search in
 * @param pattern Hexadecimal pattern string (e.g., "48 8B 05 12 34 56 78")
 * @return Address where pattern was found, or 0 if not found
 * @throws std::runtime_error if module not found or module information unavailable
 *
 * @note Wildcards are not supported in this implementation
 * @note Pattern matching is exact byte-for-byte comparison
 */
uintptr_t Mem::ScanPattern(const std::string& moduleName, const std::string& pattern) {
    uintptr_t moduleBase = GetModuleBaseAddress(moduleName);
    if (moduleBase == 0) {
        throw std::runtime_error("module was not found. Check your module name.");
    }

    std::vector<uint8_t> patternBytes;
    std::string cleanPattern = pattern;

    // Remove spaces from pattern string
    cleanPattern.erase(std::remove(cleanPattern.begin(), cleanPattern.end(), ' '), cleanPattern.end());

    // Convert hex string to bytes
    for (size_t i = 0; i < cleanPattern.length(); i += 2) {
        std::string byteString = cleanPattern.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
        patternBytes.push_back(byte);
    }

    MODULEINFO moduleInfo;
    if (!GetModuleInformation(processHandle, reinterpret_cast<HMODULE>(moduleBase), &moduleInfo, sizeof(moduleInfo))) {
        throw std::runtime_error("Failed to get module information");
    }

    auto moduleBytes = ReadBytes(moduleBase, moduleInfo.SizeOfImage);
    int offset = ScanPattern(moduleBytes, patternBytes);

    return (offset != -1) ? moduleBase + offset : 0;
}

/**
 * @brief Scans for a byte pattern within a byte array
 *
 * Performs a linear search for the needle pattern within the haystack array.
 * Uses exact byte-for-byte comparison to find matches.
 *
 * @param haystack Byte array to search in
 * @param needle Byte pattern to search for
 * @return Index where pattern was found, or -1 if not found
 *
 * @note This is a helper function used internally by the string pattern scanner
 * @note Uses simple linear search algorithm - O(n*m) complexity
 */
int Mem::ScanPattern(const std::vector<uint8_t>& haystack, const std::vector<uint8_t>& needle) {
    if (needle.empty() || haystack.size() < needle.size()) {
        return -1;
    }

    for (size_t i = 0; i <= haystack.size() - needle.size(); i++) {
        bool found = true;
        for (size_t j = 0; j < needle.size(); j++) {
            if (haystack[i + j] != needle[j]) {
                found = false;
                break;
            }
        }
        if (found) {
            return static_cast<int>(i);
        }
    }
    return -1;
}
