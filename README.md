# Memory | Windows Process Memory Manipulation Library for C++

A comprehensive, header-only C++ library for Windows process memory operations designed for educational purposes, debugging, and reverse engineering. Built with modern C++ practices and extensive error handling.

## Key Aspects

- **Header-Only Design**: Single `mem.hpp` file with complete implementation
- **Type-Safe Operations**: Strongly-typed memory read/write operations for all common data types
- **Multi-Level Pointer Chains**: Support for complex pointer traversal up to 7 levels deep
- **Pattern Scanning**: Hexadecimal byte pattern search within process modules
- **Exception-Safe**: Comprehensive error handling with descriptive runtime exceptions
- **Windows API Integration**: Efficient wrapper around Windows memory management APIs

## Features

### Process Management
- Process attachment by executable name (e.g., "notepad.exe")
- Automatic process handle management with RAII
- Module enumeration and base address retrieval
- Process privilege validation

### Memory Operations
- **Reading**: `int`, `float`, `short`, `unsigned` variants, raw bytes, Vector3, 4x4 matrices
- **Writing**: All read types plus specialized NOP instruction patching
- **Pointer Chains**: Traverse complex nested data structures with offset arrays
- **Offset Support**: All operations support optional memory offsets

### Advanced Features
- **Pattern Scanning**: Search for hexadecimal byte patterns within module memory
- **Vector3 Support**: Built-in 3D vector structure for spatial data
- **Matrix Operations**: Read 4x4 transformation matrices as float arrays
- **Code Patching**: Write NOP instructions for code modification

## Repository Structure

```
Memory/
├── Memory.sln
├── README.md
└── Memory/
    ├── Memory.vcxproj
    ├── include/
    │   └── mem.hpp        # Complete library implementation
    └── src/
        └── main.cpp       # Example: Game memory manipulation
```

## Usage Examples

### Basic Process Attachment and Memory Reading

```cpp
#include "mem.hpp"
#include <iostream>

int main() {
    try {
        // Attach to target process
        Mem mem("notepad.exe");
        
        // Get module base address
        uintptr_t baseAddr = mem.GetModuleBaseAddress("notepad.exe");
        
        // Read different data types
        int intValue = mem.ReadInt(baseAddr + 0x1000);
        float floatValue = mem.ReadFloat(baseAddr + 0x2000);
        Vector3 position = mem.ReadVector3(baseAddr + 0x3000);
        
        std::cout << "Integer: " << intValue << std::endl;
        std::cout << "Float: " << floatValue << std::endl;
        std::cout << "Position: (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
        
    } catch (const std::runtime_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### Pointer Chain Traversal

```cpp
// Example: Following a pointer chain
// [[baseAddress + 0x3C] + 0x28] + 0x4] + 0x0] + 0x68] + 0x0] + 0x550C
std::vector<int> offsets = {0x3C, 0x28, 0x4, 0x0, 0x68, 0x0, 0x550C};
uintptr_t baseAddr = mem.GetModuleBaseAddress("game.exe") + 0x0010C740;

// Method 1: Using vector of offsets
uintptr_t finalAddr = mem.ReadPointer(baseAddr, offsets);

// Method 2: Using individual parameters (up to 7 levels)
uintptr_t sameAddr = mem.ReadPointer(baseAddr, 0x3C, 0x28, 0x4, 0x0, 0x68, 0x0, 0x550C);

// Read the final value
int gameResource = mem.ReadInt(finalAddr);
```

### Memory Writing and Modification

```cpp
// Write different data types
mem.WriteInt(address, 1000);
mem.WriteFloat(address + 4, 3.14f);
mem.WriteVector3(address + 8, Vector3(10.0f, 20.0f, 30.0f));

// Write raw bytes
std::vector<uint8_t> data = {0x90, 0x90, 0x90, 0x90}; // NOP instructions
mem.WriteBytes(address, data);

// Convenient NOP writing for code patching
mem.WriteNop(address, 5); // Write 5 NOP instructions
```

### Pattern Scanning

```cpp
// Search for byte patterns in module memory
std::string pattern = "48 8B 05 12 34 56 78"; // Example x64 assembly pattern
uintptr_t patternAddr = mem.ScanPattern("game.exe", pattern);

if (patternAddr != 0) {
    std::cout << "Pattern found at: 0x" << std::hex << patternAddr << std::endl;
} else {
    std::cout << "Pattern not found" << std::endl;
}
```

### Real-World Example: Game Value Modification

The included `main.cpp` demonstrates practical usage by modifying game resources:

```cpp
// Attach to game process
Mem mem("popcapgame1.exe");
uintptr_t base = mem.GetModuleBaseAddress("popcapgame1.exe") + 0x0010C740;

// Define pointer chain to game resource
std::vector<int> offsets = {0x3C, 0x28, 0x4, 0x0, 0x68, 0x0, 0x550C};

while (true) {
    // Follow pointer chain manually with validation
    uintptr_t addr = mem.ReadUInt(base);
    for (size_t i = 0; i < offsets.size() - 1; i++) {
        if (addr == 0) break;
        addr = mem.ReadUInt(addr + offsets[i]);
    }
    
    if (addr == 0) {
        std::cout << "Start a level first!" << std::endl;
        continue;
    }
    
    // Read and modify game resource
    uintptr_t finalAddr = addr + offsets.back();
    int currentSuns = mem.ReadInt(finalAddr);
    
    std::cout << "Current suns: " << currentSuns << std::endl;
    // ... user input and modification logic
}
```

## API Reference

### Constructor
```cpp
Mem(const std::string& processName)  // Throws std::runtime_error if process not found
```

### Process Information
```cpp
HANDLE GetProcessHandle() const
DWORD GetProcessId() const  
const std::string& GetProcessName() const
bool SetProcess(const std::string& processName)
uintptr_t GetModuleBaseAddress(const std::string& moduleName)
```

### Memory Reading Operations
```cpp
// Basic types
int ReadInt(uintptr_t address [, int offset])
float ReadFloat(uintptr_t address [, int offset])
short ReadShort(uintptr_t address [, int offset])
unsigned short ReadUShort(uintptr_t address [, int offset])
unsigned int ReadUInt(uintptr_t address [, int offset])

// Complex types
Vector3 ReadVector3(uintptr_t address [, int offset])
std::vector<float> ReadMatrix4x4(uintptr_t address)
std::vector<uint8_t> ReadBytes(uintptr_t address [, int offset], size_t size)

// Pointer operations
uintptr_t ReadPointer(uintptr_t address [, int offset])
uintptr_t ReadPointer(uintptr_t address, const std::vector<int>& offsets)
uintptr_t ReadPointer(uintptr_t address, int offset1, int offset2, ...) // Up to 7 levels
```

### Memory Writing Operations
```cpp
// Basic types
bool WriteInt(uintptr_t address [, int offset], int value)
bool WriteFloat(uintptr_t address [, int offset], float value)
bool WriteShort(uintptr_t address [, int offset], short value)
bool WriteUShort(uintptr_t address [, int offset], unsigned short value)
bool WriteUInt(uintptr_t address [, int offset], unsigned int value)

// Complex types
bool WriteVector3(uintptr_t address [, int offset], const Vector3& value)
bool WriteBytes(uintptr_t address [, int offset], const std::vector<uint8_t>& data)
bool WriteNop(uintptr_t address, size_t length)
```

### Pattern Scanning
```cpp
uintptr_t ScanPattern(const std::string& moduleName, const std::string& pattern)
int ScanPattern(const std::vector<uint8_t>& haystack, const std::vector<uint8_t>& needle)
```

## Technical Details

### Compilation Requirements
- **Language**: C++11 or higher
- **Platform**: Windows (x86/x64)
- **Compiler**: Visual Studio 2017+ (MSVC), MinGW-w64, or Clang with Windows SDK
- **Dependencies**: Windows SDK for API headers

### Required Headers
```cpp
#include <windows.h>    // Core Windows API
#include <tlhelp32.h>   // Process enumeration
#include <psapi.h>      // Module enumeration
```

### Build Configuration
- **Debug/Release**: Both configurations supported
- **Architecture**: x86 and x64 builds available
- **Runtime**: Static or dynamic linking supported

### Privileges and Security
- **SeDebugPrivilege**: Required for accessing protected processes
- **Administrator Rights**: May be needed for system processes
- **Process Access Rights**: Uses `PROCESS_ALL_ACCESS` for maximum flexibility

### Error Handling
All memory operations may throw `std::runtime_error` with descriptive messages:
- Process attachment failures
- Invalid memory addresses
- Insufficient permissions
- Module not found errors
- Memory operation failures

## Limitations and Notes

- **Pattern Scanning**: Current implementation does not support wildcards (e.g., `??` or `?`)
- **Architecture**: Target process architecture must match the library build
- **Permissions**: Requires appropriate process access rights
- **Platform**: Windows-only due to Windows API dependencies

## Security and Legal Notice

This library is designed for:
- Educational purposes and learning memory management concepts
- Legitimate debugging and software analysis
- Game modding and development assistance
- Security research in controlled environments

**Important**: Always ensure you have proper authorization before accessing other processes' memory. Respect software licenses, terms of service, and applicable laws.

## Building the Project

1. **Clone/Download**: Obtain the source code
2. **Open Solution**: Load `Memory.sln` in Visual Studio
3. **Select Configuration**: Choose Debug/Release and x86/x64
4. **Build**: Compile using Build -> Build Solution
5. **Run**: Execute the resulting binary with appropriate privileges

## Version Information

- **Version**: 1.0
- **Author**: edoardohb
- **License**: MIT

## Contributing

Contributions are welcome! When submitting changes:
- Maintain the existing code style and documentation standards
- Ensure compatibility with both x86 and x64 architectures
- Add appropriate error handling for new functionality
- Update this README for any API changes

For questions, issues, or feature requests, please ensure you include relevant system information and a minimal reproducible example.