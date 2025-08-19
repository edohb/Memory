#include <iostream>
#include "mem.hpp"

// Usage example - Credits to swedz: https://www.youtube.com/watch?v=BPQ_amHTF0Q

int main() {
    Mem mem("popcapgame1.exe");
    unsigned int base = mem.GetModuleBaseAddress("popcapgame1.exe") + 0x0010C740;

    std::vector<int> offsets = { 0x3C, 0x28, 0x4, 0x0, 0x68, 0x0, 0x550C };

    while (true) {
        unsigned int addr = mem.ReadUInt(base);
        for (size_t i = 0; i < offsets.size() - 1; i++) {
            if (addr == 0) break;
            addr = mem.ReadUInt(addr + offsets[i]);
        }

        unsigned int finalAddr = addr + offsets.back();
        if (addr == 0) {
            std::cout << "Start a level first!\n";
            system("pause");
            continue;
        }

        int suns = mem.ReadInt(finalAddr);

        std::cout << "Suns: " << suns << std::endl;
        int newSuns;

        std::cout << "New suns (0 to exit): ";
        std::cin >> newSuns;

        if (newSuns <= 0) break;

        mem.WriteInt(finalAddr, newSuns);
    }
    return 0;
}
